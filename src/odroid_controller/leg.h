#pragma once

#include <assert.h>

#include <atomic>
#include <chrono>

#include "leg_control.h"
#include "motor.h"
#include "pid_utils.h"

struct LegConfiguration {
  float theta = 0.f;
  float gamma = 0.f;
  float z = 0.f;
};

class Leg {
 public:
  Leg(Motor* m_i, Motor* m_o, Motor* m_z);
  void UpdateConfig(const YAML::Node& config);
  void UpdateControl(float dt);

  Motor* GetMotorI() { return m_i_; }
  Motor* GetMotorO() { return m_o_; }
  Motor* GetMotorZ() { return m_z_; }
  const Motor* GetMotorI() const { return m_i_; }
  const Motor* GetMotorO() const { return m_o_; }
  const Motor* GetMotorZ() const { return m_z_; }

  std::string GetName() const { return name_; }
  void SetName(const std::string& n) { name_ = n; }

  float GetAngleI() const {
    return m_i_->GetAccumulatedAngle() * m_i_->GetGearRatio();
  }
  float GetAngleO() const {
    return m_o_->GetAccumulatedAngle() * m_o_->GetGearRatio();
  }
  float GetAngleZ() const {
    return m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio() - z_offset_;
  }

  float GetGamma() const {
    float angle_o = GetAngleO();
    float angle_i = GetAngleI();
    // if (fabsf(angle_o - angle_i) > M_PI) {
    //   if (angle_i < angle_o) {
    //     angle_i += 2.f * M_PI;
    //   } else {
    //     angle_o += 2.f * M_PI;
    //   }
    // }

    double gamma = NormalizeAngle(angle_i - angle_o) -
                   1.4 * m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio();
    gamma = NormalizeAngle(gamma);
    return gamma;
  }

  float GetTheta() const {
    float angle_o = GetAngleO();
    // float angle_i = GetAngleI();

    // float theta = 0.5 * (angle_o + angle_i);
    float theta =
        angle_o + 0.4 * m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio();
    theta = NormalizeAngle(theta) ;
    return theta;
  }

  void SetControl(LegControl* control) { active_control_ = control; }

  // void SetMinZ(float z) { z_min_ = z; };
  // void ZeroIORevs() {
  //   {
  //     float angle = GetMotorI()->GetAccumulatedAngle();
  //     angle = NormalizeAngle(angle);
  //     GetMotorI()->SetAccumulatedAngle(angle);
  //   }

  //   {
  //     float angle = GetMotorO()->GetAccumulatedAngle();
  //     angle = NormalizeAngle(angle);
  //     GetMotorO()->SetAccumulatedAngle(angle);
  //   }
  // }

  float GetMinZ() const { return min_z_; }
  float GetMaxZ() const { return max_z_; }
  float GetMinGamma() const { return min_gamma_; }
  float GetMaxGamma() const { return max_gamma_; }
  float GetZeroThetaOffset() const { return zero_theta_offset_; }
  float GetInitSafeZ() const { return init_safe_z_; }
  float GetInitRefTheta() const { return init_ref_theta_; }

  const PIDParams& GetThetaPIDConfig() const { return theta_pd_config_; }
  const PIDParams& GetGammaPIDConfig() const { return gamma_pd_config_; }
  const PIDParams& GetZPIDConfig() const { return z_pd_config_; }

  LegConfiguration GetState() const {
    return {GetTheta(), GetGamma(), GetAngleZ()};
  }

  void UpdateZOffset(float reference_angle) {
    z_offset_ =
        m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio() - reference_angle;
  }

 private:
  PIDParams theta_pd_config_;
  PIDParams gamma_pd_config_;
  PIDParams z_pd_config_;

  Motor* m_i_;
  Motor* m_o_;
  Motor* m_z_;

  float z_min_ = 0.0;
  float z_max_ = 1000.0;
  float theta_offset = 0.0;

  float z_offset_ = 0.0;

  std::chrono::system_clock::duration prev_update_time_;

  bool time_initialized_ = false;

  LegControl* active_control_ = nullptr;
  // LegCommandQueue queue_;
  std::string name_;

  // void ZeroGamma() {
 private:
  float min_z_ = 0.0;
  float max_z_ = 0.0;
  float min_gamma_ = 0.0;
  float max_gamma_ = 0.0;
  float zero_theta_offset_ = 0.0;
  float init_safe_z_ = 0.0;
  float init_ref_theta_ = 0.0;
};

class ThetaGammaZControl;

struct Legs {
  Leg* fr = nullptr;
  Leg* fl = nullptr;
  Leg* br = nullptr;
  Leg* bl = nullptr;

  Leg* GetLeg(int index) const {
    switch (index) {
      case 0:
        return fr;
      case 1:
        return fl;
      case 2:
        return bl;
      case 3:
        return br;
      default:
        return nullptr;
    }
  }
};