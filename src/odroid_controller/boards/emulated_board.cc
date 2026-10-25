#include "emulated_board.h"
#include <algorithm>
#include <stdexcept>

namespace boards {
EmulatedBoard::EmulatedBoard(std::unique_ptr<Transport> transport, std::shared_ptr<Clock> clock,
                             std::shared_ptr<MemoryLink> link)
    : transport_(std::move(transport)), clock_(std::move(clock)), link_(std::move(link)) {
  if (!transport_->Open()) throw std::runtime_error("cannot open emulator transport");
  Reset();
}
void EmulatedBoard::Reset() {
  // These are external sensor inputs, not MCU configuration. A reset cannot
  // cool a motor or move its shaft, even before the next plant sample arrives.
  const auto sensors = channels_;
  channels_ = {};
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].encoder = sensors[i].encoder;
    channels_[i].temperature = sensors[i].temperature;
  }
  state_ = State::PreInit;
  input_.clear(); output_.clear();
  sync_due_ = init_due_ = thermal_due_ = -1;
  synced_ = temperature_next_ = false;
  watchdog_due_ = clock_->Now() + kWatchdogTimeout;
  next_ping_ = clock_->Now() + 500000;
  next_telemetry_ = clock_->Now();
  stalled_until_ = 0;
  if (link_) generation_ = link_->generation;
}
void EmulatedBoard::WatchdogReset(bool thermal) {
  ++watchdog_resets_;
  if (thermal) ++thermal_resets_;
  if (link_) link_->Reset();
  Reset();
}
void EmulatedBoard::SetSensors(int i, uint16_t angle, uint16_t temperature) {
  channels_.at(i).encoder = angle & 4095;
  channels_.at(i).temperature = temperature;
}
double EmulatedBoard::Voltage(int i) const {
  const auto& c = channels_.at(i);
  // Unsupported direct-phase excitation is never passed off as FOC voltage.
  return state_ == State::Running && (c.availability & 1) && c.motion_type == 0
      ? c.voltage / 512.0 : 0;
}
void EmulatedBoard::String(uint8_t level, const std::string& text) {
  output_.push_back(MESSAGE_TYPE_STRING | level);
  output_.push_back(text.size());
  output_.insert(output_.end(), text.begin(), text.end());
}
void EmulatedBoard::Reply(uint8_t command, uint16_t value, size_t width) {
  output_.push_back(MESSAGE_TYPE_GET_REPLY | command);
  output_.push_back(value & 255);
  if (width == 2) output_.push_back(value >> 8);
  output_.push_back(0);  // active firmware's sequence placeholder
}
void EmulatedBoard::Process(const Bytes& b) {
  ++commands_;
  const auto cmd = GetCommand(b[0]);
  auto& c = channels_[GetMotorIndex(b[0])];
  const auto start = GetCommandLength(b[0]);
  if (IsGetMessage(b[0])) {
    if (cmd == CMD_MOTOR_VOLTAGE || cmd == CMD_MOTOR_PHASE) return; // firmware emits no reply
    if (!SupportsGet(cmd, b.size() > 1 ? b[1] : 0)) { String(0, "unsupported GET"); return; }
    switch (cmd) {
      case CMD_STATE: Reply(cmd, static_cast<uint8_t>(state_), 1); break;
      case CMD_AVAILABILITY: Reply(cmd, c.availability, 1); break;
      case CMD_SENSOR: Reply(cmd, c.electrical_zero, 2); break;
      case CMD_LIMITS: Reply(cmd, c.voltage_limit, 2); break;
      case CMD_MOTION_TYPE: Reply(cmd, c.motion_type, 1); break;
      case CMD_PWMMOD: Reply(cmd, c.centered, 1); break;
    }
    return;
  }
  switch (cmd) {
    case CMD_SYNC: sync_due_ = clock_->Now() + 2000000; break;
    case CMD_AVAILABILITY: c.availability = b[start]; break;
    case CMD_STATE: {
      auto requested = static_cast<State>(b[start]);
      String(2, std::string("CPSC") + char('A' + b[start]));
      String(2, state_ == State::PreInit ? "CSPI" : state_ == State::Running ? "CSR" :
                state_ == State::Stopped ? "CSS" : state_ == State::Init ? "CSI" : "CSE");
      if (state_ == State::PreInit && requested == State::Init) {
        state_ = State::Init;
        init_due_ = clock_->Now() + 1000; // explicit approximation of sensor/FOC initialization
      } else if ((state_ == State::Stopped && requested == State::Running) ||
                 (state_ == State::Running && requested == State::Stopped)) state_ = requested;
      break;
    }
    case CMD_MOTOR_VOLTAGE: c.voltage = I16(b.data()+start); c.motion_type = 0; break;
    case CMD_SENSOR:
      if (b[1] == SCMD_SENS_ELEC_OFFSET) c.electrical_zero = I16(b.data()+start);
      else String(0, "USCM");
      break;
    case CMD_SENSOR_LINEARIZATION:
      if (b[1] == SCMD_OFFSET) {
        c.linearization_offset = b[start];
        if (b[start+1]) c.wires_swapped = !c.wires_swapped;
      } else if (b[1] == SCMD_FACTOR && b[start] < 16) c.linearization[b[start]] = b[start+1];
      else String(0, "invalid linearization index");
      break;
    case CMD_MOTION_TYPE:
      if (b[start] < 5) c.motion_type = b[start];
      break;
    case CMD_PWMMOD:
      if (b[1] == SCMD_PWMMOD_CENTER) c.centered = b[start]; else String(0, "UPCM");
      break;
    case CMD_LIMITS:
      if (b[1] == SCMD_LIM_VOLT) c.voltage_limit = I16(b.data()+start);
      else String(0, "ULCM");
      break;
    case CMD_MOTOR_PHASE: String(0, "direct-phase plant unsupported"); break;
    default: String(0, "UCMD"); break;
  }
}
void EmulatedBoard::Flush() {
  if (output_.size() > 4096) throw std::runtime_error("emulator output overflow");
  if (output_.empty()) return;
  const int n = transport_->Write(output_.data(), output_.size());
  if (n > 0) {
    output_.erase(output_.begin(), output_.begin()+n);
    next_ping_ = clock_->Now() + 500000;
  }
}
void EmulatedBoard::Pump() {
  if (link_ && generation_ != link_->generation) Reset();
  const auto now = clock_->Now();
  // The hardware timer runs independently of firmware progress. Check expiry
  // before servicing anything, including when a stall ends between Pump calls.
  if (thermal_due_ >= 0) {
    if (now >= thermal_due_) WatchdogReset(true);
    return;  // CheckTemperature blocks until the short watchdog resets the MCU.
  }
  if (now >= watchdog_due_) { WatchdogReset(false); return; }
  if (now < stalled_until_) return;
  if (sync_due_ >= 0) {
    if (now < sync_due_) return;  // firmware blocks command parsing during sync
    output_.push_back(MESSAGE_SUBTYPE_SYNC);
    synced_ = true; sync_due_ = -1;
  }
  if (init_due_ >= 0 && now >= init_due_) {
    state_ = State::Stopped; init_due_ = -1;
    for (auto& channel : channels_) {
      channel.motion_type = 0;
      if (channel.availability & 1) channel.voltage_limit = 12 * 512 - 1;
    }
    String(2, "INIC");
  }
  uint8_t bytes[128];
  const int n = transport_->Read(bytes, sizeof(bytes));
  if (n < 0) { Reset(); return; }
  if (n > 0) input_.insert(input_.end(), bytes, bytes+n);
  while (!input_.empty() && sync_due_ < 0) {
    size_t count = CommandSize(input_[0]);
    if (input_.size() < count) break;
    Bytes command(input_.begin(), input_.begin()+count);
    input_.erase(input_.begin(), input_.begin()+count);
    Process(command);
  }
  if (sync_due_ >= 0) return;  // Tick is blocked by the firmware's 2 s sync delay.
  Flush();
  // Approximate the firmware's buffer-capacity driven telemetry. At most one
  // five-byte packet per 500 us; bandwidth/backlog can reduce this rate.
  if (synced_ && state_ != State::PreInit &&
      (channels_[0].availability & 2) && (channels_[1].availability & 2) && now >= next_telemetry_ &&
      output_.empty() && transport_->Pending() == 0) {
    const uint8_t fields = temperature_next_ ? DATA_STREAM_TEMPERATURE_BIT : DATA_STREAM_ANGLE_BIT;
    output_.push_back(MESSAGE_TYPE_DATA_STREAM | fields);
    for (const auto& channel : channels_)
      Append(output_, uint16_t(temperature_next_ ? channel.temperature : channel.encoder));
    temperature_next_ = !temperature_next_;
    next_telemetry_ = now + 500;
    Flush();
  }
  if (synced_ && output_.empty() && now >= next_ping_) {
    output_.push_back(MESSAGE_TYPE_PING);
    Flush();
  }
  // MainCritical runs after Tick. Only successful critical-loop completion
  // feeds the watchdog; serial traffic is neither necessary nor sufficient.
  if (channels_[0].temperature > 55 || channels_[1].temperature > 55)
    thermal_due_ = now + kThermalWatchdogTimeout;
  else watchdog_due_ = now + kWatchdogTimeout;
}
}  // namespace boards
