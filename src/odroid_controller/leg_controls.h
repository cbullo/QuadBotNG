#pragma once

#include <iostream>
#include <optional>

#include "leg.h"
#include "leg_control.h"
#include "pid_utils.h"

struct LegSteering {
  float steering_i = 0.f;
  float steering_o = 0.f;
  float steering_z = 0.f;
};

namespace Steering {
struct NegativeT {
} static Negative;
struct PositiveT {
} static Positive;
struct DisabledT {
} static Disabled;
};  // namespace Steering

class CalibrationLegControl : public LegControl {
 public:
  enum CalibrationMode {
    LinearizationReading,
    LinearizationValidation,
    ElectricZero,
    MotorVoltage
  };

  bool Process(Leg& leg, float dt) override;

  CalibrationMode GetCalibrationMode() const { return calibration_mode_; }
  void SetCalibrationMode(CalibrationMode mode);
  void SetMotorIndex(int index);
  void SendSetupData(Leg& leg);
  void SetRunning(bool running) {
    std::cout << "Set running: " << running << std::endl;
    running_ = running;
    step_ = 0;
    next_read_delta_ = -0.1f;
    // if (running_ && calibration_mode_ == CalibrationMode::ElectricZero) {
    //   std::cout << "Delta set" << std::endl;
    //   next_read_delta_ = 1.5f;
    // }
  }
  bool IsRunning() const { return running_; }

  int GetStep() const { return step_; }
  void SetStep(int step) { step_ = step; }

  void SetMotorControl(float control) { voltage_control_ = control; }

  uint16_t raw_angle_ = 0;

 private:
  // Motor* GetMotor(Leg& leg, int index) const;
  CalibrationMode calibration_mode_ = CalibrationMode::LinearizationReading;
  int motor_index_ = 0;
  bool running_ = false;
  int step_ = 0;
  float next_read_delta_ = 0.f;
  int zero_angle_ = 0;
  float avg_zero_angle_ = 0.f;
  float voltage_control_ = 0.f;
  float delay_ = 0.f;
};

struct LegState {
  float theta_setpoint = 0.0;
  float gamma_setpoint = 0.0;
  float z_setpoint = 0.0;

  PIDState theta_pid_state;
  PIDState gamma_pid_state;
  PIDState z_pid_state;
  // float prev_theta_error = 0.0;
  // float prev_gamma_error = 0.0;
  // float prev_z_error = 0.0;
  // float i_theta_sum = 0.0;
  // float i_gamma_sum = 0.0;
  // float i_z_sum = 0.0;

  void Reset() {
    theta_pid_state.Reset();
    gamma_pid_state.Reset();
    z_pid_state.Reset();
  }
};

class ThetaGammaZLegControl : public LegControl {
 public:
  bool Process(Leg& leg, float dt) override;

  void SetThetaSetpoint(float theta) { state_.theta_setpoint = theta; }
  void SetGammaSetpoint(float gamma) { state_.gamma_setpoint = gamma; }
  void SetZSetpoint(float z) { state_.z_setpoint = z; }

 private:
  LegState state_;

  float delay_ = -0.01f;
};

class InitializationLegControl : public LegControl {
  enum class InitializationStage {
    PreInitialization,
    DriveToMinGamma,
    DriveToSafeZ,
    // DriveToReferenceTheta,
    FindMinZ,
    DriveToMinZ,
    DriveToNextThetaRev
  };

 public:
  bool Process(Leg& leg, float dt) override;
  void Restart() { stage_ = InitializationStage::DriveToMinGamma; }

 private:
  void ChangeStage(InitializationStage new_stage);
  float reference_theta_angle_ = 0.f;
  float measured_z_angle_[9] = {0.f};
  float max_z_angle = 0.f;

  float locked_theta_ = 0.f;
  float locked_z_ = 0.f;
  float locked_gamma_ = 0.f;

  int current_theta_rev_ = 0;
  InitializationStage stage_ = InitializationStage::PreInitialization;

  LegState state_;
  float delay_ = -0.01f;
};

template <class T, class G, class Z>
bool DriveTo(Leg& leg, LegState& state, T theta, G gamma, Z z,
             const LegConfiguration& current, float dt, LegSteering& steering) {
  float tau_theta;
  bool theta_reached = Drive(theta, current.theta, state.theta_pid_state,
                             leg.GetThetaPIDConfig(), dt, tau_theta, false);
  float tau_gamma;
  bool gamma_reached = Drive(gamma, current.gamma, state.gamma_pid_state,
                             leg.GetGammaPIDConfig(), dt, tau_gamma, false);
  float tau_z;
  bool z_reached = Drive(z, current.z, state.z_pid_state, leg.GetZPIDConfig(),
                         dt, tau_z, true);

  steering.steering_i = 0.0 * tau_theta + 1.0 * tau_gamma;
  steering.steering_o = 1.0 * tau_theta - 1.0 * tau_gamma;
  steering.steering_z =
      //0.4 * tau_theta - 1.4 * tau_gamma + 
      tau_z; /*- 0.75f * tau_gamma*/
  ;

  // if (theta_reached) {
  //   std::cout << "Theta reached ";
  // }

  // if (gamma_reached) {
  //   std::cout << "Gamma reached ";
  // }

  // if (z_reached) {
  //   std::cout << "Z reached ";
  // }

  // std::cout << "Theta: "
  //           << "TRG:"
  //           << (state.theta_pid_state.last_target_set
  //                   ? std::to_string(state.theta_pid_state.last_target)
  //                   : "NONE")
  //           << " MEAN:" << state.theta_pid_state.approx_running_mean
  //           << " VAR:" << state.theta_pid_state.approx_running_variance;

  // std::cout << "Gamma: "
  //           << "TRG:"
  //           << (state.gamma_pid_state.last_target_set
  //                   ? std::to_string(state.gamma_pid_state.last_target)
  //                   : "NONE")
  //           << " MEAN:" << state.gamma_pid_state.approx_running_mean
  //           << " VAR:" << state.gamma_pid_state.approx_running_variance;

  // std::cout << "Z: "
  //           << "TRG:"
  //           << (state.z_pid_state.last_target_set
  //                   ? std::to_string(state.z_pid_state.last_target)
  //                   : "NONE")
  //           << " MEAN:" << state.z_pid_state.approx_running_mean
  //           << " VAR:" << state.z_pid_state.approx_running_variance;

  return theta_reached && gamma_reached && z_reached;
}
