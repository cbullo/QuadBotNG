#include "dashboard.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace {
std::string Quote(const std::string& s) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << c;
  }
  return out.str() + '"';
}
struct Object {
  std::string data = "{";
  void Raw(const std::string& key, const std::string& value) {
    if (data.size() > 1) data += ',';
    data += Quote(key) + ':' + value;
  }
  void String(const std::string& key, const std::string& value) { Raw(key, Quote(value)); }
  void Number(const std::string& key, double value) {
    std::ostringstream out;
    out << std::setprecision(15) << value;
    Raw(key, std::isfinite(value) ? out.str() : "null");
  }
  void Bool(const std::string& key, bool value) { Raw(key, value ? "true" : "false"); }
  std::string Finish() const { return data + '}'; }
};
using Wall = std::chrono::steady_clock;
constexpr size_t kMaxOutput = 128 * 1024;
}

Dashboard::Dashboard(int port, bool allow_control, std::string backend)
    : allow_control_(allow_control), backend_(std::move(backend)) {
  if (!port) return;
  listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0) throw std::runtime_error("dashboard socket failed");
  int reuse = 1;
  setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(port);
  if (bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) || listen(listener_, 1)) {
    close(listener_); listener_ = -1;
    throw std::runtime_error("cannot bind dashboard loopback port");
  }
}
Dashboard::~Dashboard() {
  if (client_ >= 0) close(client_);
  if (listener_ >= 0) close(listener_);
}
void Dashboard::Disconnect(RobotSystem& robot) {
  if (remote_hold_) { robot.Stop(); reason_ = "dashboard disconnected"; }
  remote_hold_ = false;
  if (client_ >= 0) close(client_);
  client_ = -1;
  input_.clear(); output_.clear(); sequence_ = 0;
}
void Dashboard::Command(RobotSystem& robot, const std::string& line) {
  Object ack;
  ack.String("type", "ack");
  bool heartbeat = false;
  try {
    if (line.empty() || line.front() != '{' || line.back() != '}')
      throw std::invalid_argument("expected a JSON object");
    // This is a local bridge protocol, deliberately flat and bounded. Reject
    // YAML extensions before using the already-linked parser for JSON input.
    if (line.find_first_of("&*!\n\r") != std::string::npos)
      throw std::invalid_argument("unsupported command syntax");
    auto c = YAML::Load(line);
    if (!c.IsMap() || c.size() > 8) throw std::invalid_argument("invalid command object");
    for (const auto& field : c)
      if (!field.first.IsScalar() || !field.second.IsScalar())
        throw std::invalid_argument("command fields must be scalar");
    auto seq = c["seq"].as<uint64_t>();
    ack.Number("seq", seq);
    if (!seq || seq <= sequence_) throw std::invalid_argument("stale command sequence");
    sequence_ = seq;
    const auto op = c["command"].as<std::string>();
    ack.String("command", op);
    heartbeat = op == "heartbeat";
    if (op == "stop") {
      robot.Stop(); remote_hold_ = false; overridden_ = true; reason_ = "operator stop";
    } else if (heartbeat) {
      if (remote_hold_) lease_ = Wall::now() + std::chrono::milliseconds(500);
    } else {
      if (!allow_control_) throw std::invalid_argument("motion disabled; use --dashboard-control");
      if (!robot.Ready() || !robot.GetPoseFault().empty())
        throw std::invalid_argument("robot not ready or pose fault latched");
      for (const auto& [name, board] : robot.boards) {
        if (!board->HasFreshEncoderFeedback(100000)) throw std::invalid_argument("stale encoders");
        for (int i = 0; i < 2; ++i)
          if (board->GetMotor(i) && (board->TemperatureAge(i) < 0 ||
              board->TemperatureAge(i) > 100000 || board->GetMotor(i)->GetTemperature() > 55))
            throw std::invalid_argument("temperature unavailable, stale or excessive");
      }
      if (op == "hold") {
        for (const auto& [name, leg] : robot.legs)
          if (leg->GetControl()) throw std::invalid_argument("stop active control before taking dashboard control");
        // Do not capture while a previous STOP is still queued in the event loop.
        robot.Update(0);
        robot.HoldCurrentPose();
        origins_.clear();
        for (const auto& [name, leg] : robot.legs) origins_[name] = leg->GetState();
        remote_hold_ = true; overridden_ = true; reason_.clear();
      } else if (op == "leg_target") {
        if (!remote_hold_) throw std::invalid_argument("hold must acquire control before setting a target");
        const auto name = c["leg"].as<std::string>();
        auto it = robot.legs.find(name);
        if (it == robot.legs.end()) throw std::invalid_argument("unknown leg");
        auto& leg = *it->second;
        const LegConfiguration target{c["theta"].as<float>(), c["gamma"].as<float>(), c["z"].as<float>()};
        const auto origin = origins_.at(name);
        if (!std::isfinite(target.theta) || !std::isfinite(target.gamma) || !std::isfinite(target.z) ||
            std::abs(target.theta-origin.theta) > 0.15 || std::abs(target.gamma-origin.gamma) > 0.15 ||
            std::abs(target.z-origin.z) > 0.15 || target.gamma < leg.GetMinGamma() ||
            target.gamma > leg.GetMaxGamma() || target.z < leg.GetMinZ() || target.z > leg.GetMaxZ())
          throw std::invalid_argument("target outside configured limits or 0.15 rad capture envelope");
        bool applied = false;
        for (auto& control : robot.hold_controls) if (leg.GetControl() == &control) {
          control.SetThetaSetpoint(target.theta); control.SetGammaSetpoint(target.gamma);
          control.SetZSetpoint(target.z); applied = true;
        }
        if (!applied) throw std::invalid_argument("dashboard no longer owns this leg");
      } else throw std::invalid_argument("unknown command; supported: stop, hold, leg_target");
      lease_ = Wall::now() + std::chrono::milliseconds(500);
    }
    ack.Bool("accepted", true);
    ack.String("message", "accepted by host; inspect telemetry for resulting state");
  } catch (const std::exception& e) {
    ack.Bool("accepted", false); ack.String("message", e.what()); heartbeat = false;
  }
  if (!heartbeat) output_ += ack.Finish() + '\n';
}

std::string Dashboard::Snapshot(const RobotSystem& robot, double time) const {
  Object out, motors, legs, boards;
  out.String("type", "state"); out.String("backend", backend_);
  out.Number("time", time); out.Bool("ready", robot.Ready());
  out.Bool("motion_enabled", allow_control_); out.Bool("remote_hold", remote_hold_);
  out.String("pose_fault", robot.GetPoseFault()); out.String("dashboard_reason", reason_);
  for (const auto& [name, motor] : robot.motors) {
    Object m;
    const auto* b = motor->GetController();
    const int i = motor->GetIndex();
    m.String("board", b->GetName()); m.Number("channel", i);
    m.Number("encoder_count", motor->GetRawAngle()); m.Number("rotor_rad", motor->GetAccumulatedAngle());
    m.Number("temperature_c", motor->GetTemperature());
    m.Number("encoder_age_ms", b->EncoderAge(i) < 0 ? -1 : b->EncoderAge(i)/1000.0);
    m.Number("temperature_age_ms", b->TemperatureAge(i) < 0 ? -1 : b->TemperatureAge(i)/1000.0);
    m.Number("voltage_command_v", b->LastVoltageCommand(i));
    motors.Raw(name, m.Finish());
  }
  for (const auto& [name, leg] : robot.legs) {
    Object l;
    l.Number("theta", leg->GetTheta()); l.Number("gamma", leg->GetGamma()); l.Number("z", leg->GetAngleZ());
    std::string mode = leg->GetControl() ? "other" : "stopped";
    for (const auto& control : robot.hold_controls) if (leg->GetControl() == &control) {
      mode = "leg_hold";
      const auto t = control.GetTarget();
      l.Number("target_theta", t.theta); l.Number("target_gamma", t.gamma); l.Number("target_z", t.z);
    }
    for (const auto& control : robot.rotor_hold_controls) if (leg->GetControl() == &control) {
      mode = "rotor_hold";
      for (int i = 0; i < 3; ++i) l.Number(std::string("target_rotor_") + "IOZ"[i], control.GetTarget()[i]);
    }
    l.String("mode", mode); legs.Raw(name, l.Finish());
  }
  for (const auto& [name, board] : robot.boards) {
    Object b;
    b.Bool("connected", board->IsConnected()); b.Bool("ready", board->IsReady());
    b.String("error", board->GetError()); b.Number("pending_commands", board->PendingCommands());
    b.Number("state_command", board->LastStateCommand()); boards.Raw(name, b.Finish());
  }
  out.Raw("motors", motors.Finish()); out.Raw("legs", legs.Finish()); out.Raw("boards", boards.Finish());
  return out.Finish();
}

void Dashboard::Pump(RobotSystem& robot, double time) {
  if (listener_ < 0) return;
  if (remote_hold_ && (Wall::now() >= lease_ || !robot.GetPoseFault().empty())) {
    robot.Stop(); remote_hold_ = false; reason_ = "control lease expired or pose fault";
  }
  // Accept at most one connection per tick. Additional bridges cannot take over.
  int incoming = accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
  if (incoming >= 0) {
    if (client_ >= 0) close(incoming);
    else { client_ = incoming; next_snapshot_ = time; }
  }
  if (client_ < 0) return;
  char buffer[2048];
  const auto n = recv(client_, buffer, sizeof(buffer), MSG_DONTWAIT);
  if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
    Disconnect(robot); return;
  }
  if (n > 0) input_.append(buffer, n);
  if (input_.size() > 8192) { Disconnect(robot); return; }
  for (int budget = 0; budget < 4; ++budget) {
    const auto end = input_.find('\n');
    if (end == std::string::npos) break;
    if (end > 1024) { Disconnect(robot); return; }
    Command(robot, input_.substr(0, end)); input_.erase(0, end+1);
  }
  if (time >= next_snapshot_) {
    // A slow reader drops whole telemetry snapshots, never partial JSON lines.
    if (output_.size() < 16384) output_ += Snapshot(robot, time) + '\n';
    next_snapshot_ = time + 0.02;
  }
  if (output_.size() > kMaxOutput) { Disconnect(robot); return; }
  if (!output_.empty()) {
    const auto written = send(client_, output_.data(), output_.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (written > 0) output_.erase(0, written);
    else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) Disconnect(robot);
  }
}
