#pragma once

#include <math.h>

#include <vector>

#include "angle.h"

#if QUADBOT_SIMULATOR
#include "gazebo_driver.h"
using ControllerType = GazeboDriver;
#else
#include "bldc_driver_board.h"
using ControllerType = BLDCDriverBoard;
#endif


class Motor {
 public:
  static constexpr double kAS5600ToRadians = M_PI / 2048.0;
  static constexpr double kRadiansToAS5600 = 2048.0 / M_PI;

  Motor(ControllerType* controller, int motor_index);
  void UpdateConfig(const YAML::Node& config);
  uint16_t GetRawAngle() const { return raw_angle_; }

  void SetZeroAngle(uint16_t zero_angle);
  inline double GetAccumulatedAngle() const {
    return accumulated_angle_ /*- zero_angle_*/;
  }
  double GetAngleDelta() const {
    return ClosestAngle(prev_offset_angle_, offset_angle_);
  }
  double GetGearRatio() const { return gear_ratio_; }
  int GetIndex() const { return motor_index_; }

  ControllerType* GetController() const { return controller_; }

  void SetName(const std::string& name) { name_ = name; }
  std::string GetName() const { return name_; }
  int GetDirection() const { return direction_; }

  void SetCurrentTemperature(uint16_t temp) { current_temp_ = temp; }
  void SetCurrentRawAngle(uint16_t angle) {
    current_raw_angle_ = angle;
    angle_received_ = true;
  }

  void Update();

  static const int linearization_factors_count = 16;
  uint8_t sensor_linearization_offset_ = 0;
  std::vector<uint8_t> sensor_linearization_coeffs_;
  int electric_zero_angle_ = 0.0;
  uint8_t swap_wires_ = 0;

 private:
  void UpdateAngle(uint16_t new_angle);

  ControllerType* controller_;
  int motor_index_;

  uint16_t raw_angle_ = 0.0;
  float zero_angle_ = 0.0;

  float offset_angle_ = 0.0;
  float prev_offset_angle_ = 0.0;
  float accumulated_angle_ = 0.0;
  float gear_ratio_ = 1.0;
  int direction_ = 1;
  bool first_update = true;

  std::atomic<uint16_t> current_temp_;
  std::atomic<uint16_t> current_raw_angle_;
  bool angle_received_ = false;

  float sensor_angle_;
  float accumulated_normalized_angle_;

  std::string name_;
};
