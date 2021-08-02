#include "motor.h"

#include "angle.h"

Motor::Motor(ControllerBoard* controller, int motor_index) {
  controller_ = controller;
  motor_index_ = motor_index;
}

void Motor::UpdateConfig(const YAML::Node& config) {
  //name_ = config.Tag();
  direction_ = config["direction"].as<int>();
  gear_ratio_ = 1.0 / config["gear_ratio_inv"].as<double>();
}

static int foo = 80000;

void Motor::UpdateRawAngle(uint16_t new_angle) {
  //  if (foo-- > 0) {
  //    std::cout << name_ << " " << new_angle << std::endl;
  //  }
  
  if (direction_ == -1) {
    new_angle = 4096 - new_angle;
  }

  if (first_update) {
    offset_angle_ = new_angle * kAS5600ToRadians;
    zero_angle_ = offset_angle_;
    first_update = false;
    raw_angle_ = new_angle;
    return;
  }

  double new_angle_rad = new_angle * kAS5600ToRadians;
  //new_angle_rad -= zero_angle_;

  prev_offset_angle_ = offset_angle_;
  offset_angle_ = new_angle_rad;

  double angle_diff = ClosestAngle(prev_offset_angle_, offset_angle_);
  accumulated_angle_ += angle_diff;
  raw_angle_ = new_angle;
}

void Motor::SetZeroAngle(uint16_t zero_angle) {
  if (direction_ == -1) {
    zero_angle = 4096 - zero_angle;
  }
  //zero_angle_ = zero_angle * kAS5600ToRadians;
  first_update = true;
  accumulated_angle_ = 0.0;
}