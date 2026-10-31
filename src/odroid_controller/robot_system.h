#pragma once
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include "controller.h"

class ScriptInput final : public EventNode {
 public:
  ScriptInput() { always_active_ = true; }
  void Send(EventId id, float value = 0) { OutputEvent({id, value}); }
  void ProcessInputEvents(const std::deque<ControlEvent>&) override {}
};

class RobotSystem {
 public:
  struct ImuSample {
    double time = 0;
    bool valid = false;
    std::array<double, 3> angular_velocity_rad_s{}, specific_force_m_s2{};
  };
  // Separate chassis sensor, not one of the motor-board protocol channels.
  // Absent until a backend supplies it; serial hardware has no IMU driver yet.
  std::optional<ImuSample> imu;
  using Factory = std::function<std::unique_ptr<BLDCDriverBoard>(const std::string&, const YAML::Node&)>;
  RobotSystem(const YAML::Node& config, Factory factory, bool joystick = false);
  void Connect();
  bool Ready() const;
  void Pump();
  void Update(float dt);
  const std::string& GetPoseFault() const { return pose_fault_; }
  // Explicit known-pose bring-up: preserves production leg PID/mixing, skips homing.
  void HoldCurrentPose();
  void HoldRotorPose(double p, double d, int voltage_sign, double voltage_limit);
  // Replay the production Start/Confirm sequence with feedback fault monitoring.
  void StartInitialization();
  void SetStandingTargets(const std::array<LegConfiguration, 4>& targets);
  bool InitializationComplete() const { return walk_->InitializationComplete(); }
  void Stop();
  void AbortPoseMotion(const std::string& reason) { StopForPoseFault(reason); }
  std::map<std::string, std::unique_ptr<BLDCDriverBoard>> boards;
  std::map<std::string, std::unique_ptr<Motor>> motors;
  std::map<std::string, std::unique_ptr<Leg>> legs;
  std::array<ThetaGammaZLegControl, 4> hold_controls;
  std::array<RotorPoseLegControl, 4> rotor_hold_controls;
  std::shared_ptr<ScriptInput> input;
  std::unique_ptr<Controller> controller;
 private:
  void StopForPoseFault(const std::string& reason);
  std::shared_ptr<WalkBehavior> walk_;
  bool pose_control_active_ = false;
  std::string pose_fault_;
};
