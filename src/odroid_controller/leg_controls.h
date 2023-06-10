#pragma once

#include <iostream>

#include "leg_control.h"

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

class ThetaGammaZLegControl : public LegControl {
 public:
  bool Process(Leg& leg, float dt) override;

  void SetThetaSetpoint(float theta) { state_.theta_setpoint = theta; }
  void SetGammaSetpoint(float gamma) { state_.gamma_setpoint = gamma; }
  void SetZSetpoint(float z) { state_.z_setpoint = z; }

 private:
  struct State {
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
  };
  State state_;

  float delay_ = -0.01f;
};

class InitializationLegControl : public LegControl {
 enum class InitializationStage {
  PreInitialization,
  DriveToMinGamma,
  DriveToSafeZ,
  DriveToReferenceTheta,
  DriveToMinZ
 };
 public:
  bool Process(Leg& leg, float dt) override;

 private:
  float reference_theta_angle = 0.f;
  float measured_z_angles[9] = {0.f};
  float max_z_angle = 0.f;

  int current_theta_rev = 0;
  InitializationStage stage_;
};
