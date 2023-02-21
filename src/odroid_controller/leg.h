#pragma once

#include <assert.h>

#include <atomic>

#include "leg_control.h"
#include "motor.h"

struct PDParams {
  float p;
  float d;
  float i;
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

  float GetAngleI() const {
    return m_i_->GetAccumulatedAngle() * m_i_->GetGearRatio();
  }
  float GetAngleO() const {
    return m_o_->GetAccumulatedAngle() * m_o_->GetGearRatio();
  }
  float GetAngleZ() const {
    return m_z_->GetAccumulatedAngle() * m_z_->GetGearRatio();
  }

  float GetGamma() const {
    float angle_o = GetAngleO();
    float angle_i = GetAngleI();
    if (fabsf(angle_o - angle_i) > M_PI) {
      if (angle_i < angle_o) {
        angle_i += 2.f * M_PI;
      } else {
        angle_o += 2.f * M_PI;
      }
    }

    return 0.5 * (angle_o - angle_i);// + 1.5 * (GetAngleZ());
  }

  float GetTheta() const {
    float angle_o = GetAngleO();
    float angle_i = GetAngleI();
    if (fabsf(angle_o - angle_i) > M_PI) {
      if (angle_i < angle_o) {
        angle_i += 2.f * M_PI;
      } else {
        angle_o += 2.f * M_PI;
      }
    }
    
    float gamma = 0.5 * (angle_o + angle_i);
    gamma = NormalizeAngle(gamma);

    return gamma;// - 0.75 * (GetAngleZ());
  }

  void SetControl(LegControl* control) { active_control_ = control; }

  void SetMinZ(float z) { z_min_ = z; };

  PDParams theta_pd_config;
  PDParams gamma_pd_config;
  PDParams z_pd_config;

 private:
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
};

class ThetaGammaZControl;

struct Legs {
  Leg* fr = nullptr;
  Leg* fl = nullptr;
  Leg* br = nullptr;
  Leg* bl = nullptr;

  Leg* GetLeg(int index) {
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