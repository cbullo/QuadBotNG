#include "motor.h"

#include "angle.h"

Motor::Motor(ControllerType* controller, int motor_index) {
  controller_ = controller;
  motor_index_ = motor_index;
}

void Motor::UpdateConfig(const YAML::Node& config) {
  // name_ = config.Tag();
  direction_ = config["encoder_direction"].as<int>();
  gear_ratio_ = 1.0 / config["gear_ratio_inv"].as<double>();
  sensor_linearization_offset_ = config["linearization_offset"].as<uint8_t>();
  sensor_linearization_coeffs_ =
      config["linearization_coeffs"].as<std::vector<uint8_t>>();
  electric_zero_angle_ = config["electric_zero_offset"].as<int>();
  swap_wires_ = config["swap_wires"].as<uint8_t>();
}

void Motor::Update() {
  if (angle_received_) {
    UpdateAngle(current_raw_angle_);
    angle_received_ = false;
  }
}

void Motor::UpdateAngle(uint16_t new_angle) {
  if (first_update) {
    offset_angle_ = new_angle * kAS5600ToRadians;
    prev_offset_angle_ = offset_angle_;
    zero_angle_ = offset_angle_;
    first_update = false;
    raw_angle_ = new_angle;
    // std::cout << "Zero angle set: " << zero_angle_ << std::endl;
    // std::cout << "Accumulated angle: " << GetAccumulatedAngle() << std::endl;
    return;
  }

  double new_angle_rad = new_angle * kAS5600ToRadians;

  prev_offset_angle_ = offset_angle_;
  offset_angle_ = new_angle_rad;

  // std::cout << prev_offset_angle_ << " " << offset_angle_ << std::endl;

  double angle_diff = ClosestAngle(prev_offset_angle_, offset_angle_);
  accumulated_angle_ += angle_diff;
  raw_angle_ = new_angle;

  // std::cout << "Accumulated angle: " << GetAccumulatedAngle() << std::endl;
}

// void Motor::SetZeroAngle(uint16_t zero_angle) {
//   if (direction_ == -1) {
//     zero_angle = 4096 - zero_angle;
//   }
//   // zero_angle_ = zero_angle * kAS5600ToRadians;
//   first_update = true;
//   accumulated_angle_ = 0.0;
// }