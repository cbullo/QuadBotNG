#pragma once

#include <math.h>

#include <vector>

#include "angle.h"
#include "bldc_driver_board.h"

class Motor {
 public:
  static constexpr double kAS5600ToRadians = M_PI / 2048.0;
  static constexpr double kRadiansToAS5600 = 2048.0 / M_PI;

  Motor(BLDCDriverBoard* controller, int motor_index);
  void UpdateConfig(const YAML::Node& config);
  uint16_t GetRawAngle() const { return raw_angle_; }
  void UpdateRawAngle(uint16_t new_angle);
  void SetZeroAngle(uint16_t zero_angle);
  inline double GetAccumulatedAngle() const {
    return accumulated_angle_ /*- zero_angle_*/;
  }
  double GetAngleDelta() const {
    return ClosestAngle(prev_offset_angle_, offset_angle_);
  }
  double GetGearRatio() const { return gear_ratio_; }
  int GetIndex() const { return motor_index_; }

  BLDCDriverBoard* GetController() const { return controller_; }

  void SetName(const std::string& name) { name_ = name; }
  std::string GetName() const { return name_; }
  int GetDirection() const { return direction_; }

  static const int linearization_factors_count = 16;
  uint8_t sensor_linearization_offset_ = 0;
  std::vector<uint8_t> sensor_linearization_coeffs_;

 private:
  BLDCDriverBoard* controller_;
  int motor_index_;

  uint16_t raw_angle_ = 0.0;
  double zero_angle_ = 0.0;

  double offset_angle_ = 0.0;
  double prev_offset_angle_ = 0.0;
  double accumulated_angle_ = 0.0;
  double gear_ratio_ = 1.0;
  int direction_ = 1;
  bool first_update = true;
  std::string name_;
};
