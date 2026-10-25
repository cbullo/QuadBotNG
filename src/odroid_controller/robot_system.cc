#include "robot_system.h"
#include "joystick_input.h"
#include <stdexcept>
#include <set>

RobotSystem::RobotSystem(const YAML::Node& config, Factory factory, bool joystick) {
  for (const auto& entry : config["controllers"]) {
    auto name = entry.first.as<std::string>();
    auto board = factory(name, entry.second);
    board->SetName(name); board->UpdateConfig(entry.second);
    boards.emplace(name, std::move(board));
  }
  std::set<std::pair<std::string,int>> bindings;
  for (const auto& entry : config["motors"]) {
    const auto name = entry.first.as<std::string>();
    const auto c = entry.second;
    auto board_name = c["controller"].as<std::string>();
    int index = c["index"].as<int>();
    if (index < 0 || index > 1 || !bindings.emplace(board_name,index).second)
      throw std::runtime_error("duplicate or invalid board channel");
    auto* board = boards.at(board_name).get();
    auto motor = std::make_unique<Motor>(board,index);
    motor->SetName(name); motor->UpdateConfig(c); board->SetMotor(index,motor.get());
    motors.emplace(name,std::move(motor));
  }
  for (const auto& entry : config["legs"]) {
    const auto c = entry.second;
    auto leg = std::make_unique<Leg>(motors.at(c["motor_I"].as<std::string>()).get(),
                                   motors.at(c["motor_O"].as<std::string>()).get(),
                                   motors.at(c["motor_Z"].as<std::string>()).get());
    auto name = entry.first.as<std::string>();
    leg->SetName(name); leg->UpdateConfig(c); legs.emplace(name,std::move(leg));
  }
  Legs mapping;
  mapping.fr = legs.at("leg_FR").get(); mapping.fl = legs.at("leg_FL").get();
  mapping.br = legs.at("leg_BR").get(); mapping.bl = legs.at("leg_BL").get();
  std::vector<BLDCDriverBoard*> board_ptrs;
  for (auto& [name,board] : boards) board_ptrs.push_back(board.get());
  auto stopped = std::make_shared<StoppedBehavior>();
  auto estopped = std::make_shared<EStoppedBehavior>();
  auto walk = walk_ = std::make_shared<WalkBehavior>();
  controller = std::make_unique<Controller>(mapping,board_ptrs,stopped,estopped);
  stopped->SetController(controller.get()); estopped->SetController(controller.get());
  controller->AttachBehavior(walk);
  controller->SubscribeToTick(walk.get());
  controller->SubscribeToEvent(walk.get(),EventId::kControlEventConfirm);
  stopped->SetPreviousBehavior(walk.get()); stopped->Activate();
  input = std::make_shared<ScriptInput>(); controller->AttachEventNode(input);
  if (joystick) {
    auto device = std::make_shared<JoystickInput>("/dev/input/js0");
    device->Init();
    auto stop_scheme = std::make_shared<StoppedScheme>();
    auto walk_scheme = std::make_shared<WalkScheme>();
    auto estop_scheme = std::make_shared<EStoppedScheme>();
    device->SetScheme(stop_scheme);
    stopped->AttachActivationCallback([device,stop_scheme](const Behavior&) { device->SetScheme(stop_scheme); });
    walk->AttachActivationCallback([device,walk_scheme](const Behavior&) { device->SetScheme(walk_scheme); });
    estopped->AttachActivationCallback([device,estop_scheme](const Behavior&) { device->SetScheme(estop_scheme); });
    controller->AttachEventNode(device); controller->SubscribeToTick(device.get());
  }
}
void RobotSystem::Connect() {
  for (auto& [name,board] : boards)
    if (!board->Connect()) throw std::runtime_error(name + ": " + board->GetError());
}
bool RobotSystem::Ready() const {
  for (const auto& [name,board] : boards) if (!board->IsReady()) return false;
  return true;
}
void RobotSystem::Pump() {
  for (auto& [name,board] : boards) {
    board->Tick();
    if (!board->GetError().empty()) {
      if (pose_control_active_) StopForPoseFault(name + ": " + board->GetError());
      // A latched pose fault has already requested a stop of every board. Keep
      // pumping healthy peers; do not abort physics or re-enable a rebooted MCU.
      if (pose_fault_.empty()) throw std::runtime_error(name + ": " + board->GetError());
    }
  }
}
void RobotSystem::StopForPoseFault(const std::string& reason) {
  if (!pose_fault_.empty()) return;
  pose_fault_ = reason;
  std::cout << "Pose control stopped: " << reason << '\n';
  Stop();
}
void RobotSystem::Update(float dt) {
  if (pose_control_active_) {
    for (const auto& [name,board] : boards) {
      // Fresh encoder samples are required even if strings or pings still
      // arrive. The firmware watchdog is independent and remains unchanged.
      if (!board->HasFreshEncoderFeedback(100000)) {
        StopForPoseFault(name + ": encoder feedback missing for 100 ms");
        break;
      }
      bool overheated = false;
      for (int i = 0; i < 2; ++i)
        if (auto* motor = board->GetMotor(i)) overheated |= motor->GetTemperature() > 55;
      if (overheated) {
        StopForPoseFault(name + ": over-temperature feedback");
        break;
      }
    }
  }
  controller->Update(dt);
  if (pose_control_active_ && !walk_->GetInitializationFault().empty())
    StopForPoseFault(walk_->GetInitializationFault());
}
void RobotSystem::HoldCurrentPose() {
  if (!pose_fault_.empty()) throw std::runtime_error("pose fault is latched: " + pose_fault_);
  if (!Ready()) throw std::runtime_error("boards not initialized");
  for (auto& [name,board] : boards) board->SendSetCommand(0,CMD_STATE,uint8_t(3));
  for (int i = 0; i < 4; ++i) {
    auto* leg = controller->GetLegs().GetLeg(i);
    auto& hold = hold_controls[i]; hold = ThetaGammaZLegControl();
    hold.SetThetaSetpoint(leg->GetTheta()); hold.SetGammaSetpoint(leg->GetGamma());
    hold.SetZSetpoint(leg->GetAngleZ()); leg->SetControl(&hold);
  }
  pose_control_active_ = true;
}
void RobotSystem::Stop() {
  pose_control_active_ = false;
  input->Send(EventId::kControlEventStop);
}
void RobotSystem::StartInitialization() {
  if (!pose_fault_.empty()) throw std::runtime_error("pose fault is latched: " + pose_fault_);
  if (!Ready()) throw std::runtime_error("boards not initialized");
  input->Send(EventId::kControlEventStart);
  // Confirm must be distributed after WalkBehavior has become active.
  controller->Update(0);
  input->Send(EventId::kControlEventConfirm);
  pose_control_active_ = true;
}
void RobotSystem::HoldRotorPose(double p, double d, int voltage_sign, double voltage_limit) {
  if (!pose_fault_.empty()) throw std::runtime_error("pose fault is latched: " + pose_fault_);
  if (!Ready()) throw std::runtime_error("boards not initialized");
  for (int i = 0; i < 4; ++i) {
    auto* leg = controller->GetLegs().GetLeg(i);
    rotor_hold_controls[i].Capture(*leg,p,d,voltage_sign,voltage_limit);
  }
  for (auto& [name,board] : boards) board->SendSetCommand(0,CMD_STATE,uint8_t(3));
  for (int i = 0; i < 4; ++i)
    controller->GetLegs().GetLeg(i)->SetControl(&rotor_hold_controls[i]);
  pose_control_active_ = true;
}

void RobotSystem::SetStandingTargets(const std::array<LegConfiguration, 4>& targets) {
  if (!pose_fault_.empty() || !pose_control_active_)
    throw std::logic_error("standing requires active fault-free homing");
  walk_->SetStandingTargets(targets);
}
