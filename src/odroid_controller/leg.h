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
    auto z =
        m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio() - zero_offset_z_;
    return z;
  }

  float GetZGammaDir() const { return z_gamma_dir_; }
  float GetGammaDir() const { return gamma_dir_; }
  float GetThetaDir() const { return theta_dir_; }

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

    // double gamma = NormalizeAngle(angle_i - angle_o) -
    //                1.4 * m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio();

    float gamma = (angle_o - angle_i) -
                  GetZGammaDir() * 1.33333333f * GetAngleZ() -
                  zero_offset_gamma_;
    // gamma = NormalizeAngle(gamma);
    return gamma;
  }

  float GetTheta() const {
    float angle_o = GetAngleO();
    float angle_i = GetAngleI();

    float theta = -0.5 * (angle_o + angle_i);
    // float theta =
    //     angle_o + 0.4 * m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio();
    theta = theta - zero_offset_theta_;
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
  float GetRefTheta() const { return ref_theta_; }

  // float GetZeroThetaOffset() const { return zero_theta_offset_; }
  // float GetInitSafeZ() const { return init_safe_z_; }
  // float GetInitRefTheta() const { return init_ref_theta_; }

  const PIDParams& GetThetaPIDConfig() const { return theta_pd_config_; }
  const PIDParams& GetGammaPIDConfig() const { return gamma_pd_config_; }
  const PIDParams& GetZPIDConfig() const { return z_pd_config_; }

  LegConfiguration GetState() const {
    return {GetTheta(), GetGamma(), GetAngleZ()};
  }

  void SetGammaOffset(float value) { zero_offset_gamma_ = value; }
  void SetThetaOffset(float value) { zero_offset_theta_ = value; }
  void SetZOffset(float value) { zero_offset_z_ = value; }

  void UpdateZAngle() {
    // auto reference_angle =
    //     m_z_->GetRawAngle() * Motor::kAS5600ToRadians * m_z_->GetGearRatio();
    // auto angle = reference_angle;  //= fmodf(reference_angle, (2.f/9.f) *
    // M_PI);

    // const auto period = (2.f / 9.f) * M_PI;

    // while (angle > period) {
    //   angle -= period;
    // }

    // while (angle <= -period) {
    //   angle += period;
    // }

    // auto angle = reference_angle + GetMaxZ();
    // m_z_->SetAccumulatedAngle(angle / m_z_->GetGearRatio());

    auto z = GetAngleZ();

    const auto period = (2.f / 9.f) * M_PI;

    auto offset =
        ref_z_ + floorf(z / period) * period - floorf(max_z_ / period) * period;

    SetZOffset(offset);
  }

  void UpdateGammaOffset() {
    auto gamma = GetGamma();

    // const auto period = (2.f / 9.f) * M_PI;
    // auto cycles = floorf(gamma / period);

    SetGammaOffset(GetGammaDir() > 0 ? gamma - GetMinGamma()
                                     : gamma - GetMaxGamma());
  }

  void UpdateThetaOffset() {
    auto theta = GetTheta();

    // const auto period = (2.f / 9.f) * M_PI;
    // auto cycles = floorf(gamma / period);

    SetThetaOffset(theta - GetRefTheta());
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

  std::chrono::system_clock::duration prev_update_time_;

  bool time_initialized_ = false;

  LegControl* active_control_ = nullptr;
  // LegCommandQueue queue_;
  std::string name_;

  // void ZeroGamma() {
 private:
  float min_z_ = 0.0;
  float max_z_ = 0.0;
  float ref_z_ = 0.0;
  float zero_offset_z_ = 0.0;

  float min_gamma_ = 0.0;
  float max_gamma_ = 0.0;
  float zero_offset_gamma_ = 0.0;

  float ref_theta_ = 0.0;
  float zero_offset_theta_ = 0.0;

  float z_gamma_dir_ = 1.0;
  float gamma_dir_ = 1.0;
  float theta_dir_ = 1.0;
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