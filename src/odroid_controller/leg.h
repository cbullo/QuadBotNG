#pragma once

#include <assert.h>

#include <atomic>

#include "motor.h"

struct PDParams {
  double p;
  double d;
  double i;
};

class LegCommand;

class Leg {
 public:
  Leg(Motor* m1, Motor* m2, Motor* tilt_motor);
  void UpdateConfig(const YAML::Node& config);
  void UpdateControl();

  Motor* GetMotorF() { return m_f_; }
  Motor* GetMotorR() { return m_r_; }
  Motor* GetMotorZ() { return m_z_; }
  const Motor* GetMotorF() const { return m_f_; }
  const Motor* GetMotorR() const { return m_r_; }
  const Motor* GetMotorZ() const { return m_z_; }

  double GetAngleF() const {
    return m_f_->GetAccumulatedAngle() * m_f_->GetGearRatio();
  }
  double GetAngleR() const {
    return m_r_->GetAccumulatedAngle() * m_r_->GetGearRatio();
  }
  double GetAngleZ() const {
    return m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio();
  }

  double GetTheta() const {
    return 0.5 * (GetAngleF() + GetAngleR()) /*- theta_offset*/;
  }

  double GetGamma() const { return 0.5 * (GetAngleR() - GetAngleF()) - 0.75 * (GetAngleZ() - z_min_); }

  void SetCommand(LegCommand* command) { active_command_ = command; }

  void SetMinZ(double z) { z_min_ = z; };

  PDParams theta_pd_config;
  PDParams gamma_pd_config;
  PDParams z_pd_config;

 private:
  Motor* m_f_;
  Motor* m_r_;
  Motor* m_z_;

  double z_min_ = 0.0;
  double z_max_ = 1000.0;

  double theta_offset = 0.0;

  std::chrono::system_clock::duration prev_update_time_;

  bool time_initialized_ = false;

  LegCommand* active_command_;
  // LegCommandQueue queue_;

  
};

class ThetaGammaZCommand;

class LegCommand {
 public:
  virtual bool Process(Leg& leg, double dt) = 0;
};



class ThetaGammaZCommand : public LegCommand {
 public:
  struct State {
    double theta_setpoint = 0.0;
    double gamma_setpoint = 0.0;
    double z_setpoint = 0.0;

    double prev_theta_error = 0.0;
    double prev_gamma_error = 0.0;
    double prev_z_error = 0.0;
    double i_theta_sum = 0.0;
    double i_gamma_sum = 0.0;
    double i_z_sum = 0.0;
  };
  // struct Config {
  //   PDParams theta_pd_config;
  //   PDParams gamma_pd_config;
  //   PDParams z_pd_config;
  // };
  bool Process(Leg& leg, double dt) override;

  void SetThetaSetpoint(double theta) { state_.theta_setpoint = theta; }
  void SetGammaSetpoint(double gamma) { state_.gamma_setpoint = gamma; }
  void SetZSetpoint(double z) { state_.z_setpoint = z; }

 private:
  State state_;
  //Config* config_;
};
