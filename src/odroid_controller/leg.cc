#include "leg.h"

#include <chrono>

#include "bldc_driver_board.h"
#include "motor.h"

Leg::Leg(Motor* m_i, Motor* m_o, Motor* m_z) {
  m_i_ = m_i;
  m_o_ = m_o;
  m_z_ = m_z;

  // theta_setpoint_ = 0;
  // gamma_setpoint_ = 0;

  // i_theta_sum_ = 0.0;
  // i_gamma_sum_ = 0.0;
}

void Leg::UpdateConfig(const YAML::Node& config) {
  theta_pd_config.p = config["theta_p"].as<double>();
  theta_pd_config.i = config["theta_i"].as<double>();
  theta_pd_config.d = config["theta_d"].as<double>();
  gamma_pd_config.p = config["gamma_p"].as<double>();
  gamma_pd_config.i = config["gamma_i"].as<double>();
  gamma_pd_config.d = config["gamma_d"].as<double>();
  z_pd_config.p = config["z_p"].as<double>();
  z_pd_config.i = config["z_i"].as<double>();
  z_pd_config.d = config["z_d"].as<double>();
}

void Leg::UpdateControl(float dt) {
  // auto now = std::chrono::system_clock::now().time_since_epoch();
  // auto seconds = std::chrono::duration<double>(now).count();

  // if (!time_initialized_) {
  //   prev_update_time_ = std::chrono::system_clock::now().time_since_epoch();
  //   time_initialized_ = true;
  //   return;
  // }

  // std::chrono::duration<double> elapsed = now - prev_update_time_;
  // auto dt = elapsed.count();
  if (active_control_) {
    if (active_control_->Process(*this, dt)) {
      
    } else {
      // GetMotorI()->GetController()->SendSetCommand(
      //     GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE, static_cast<int16_t>(0));
      // GetMotorO()->GetController()->SendSetCommand(
      //     GetMotorO()->GetIndex(), CMD_MOTOR_VOLTAGE, static_cast<int16_t>(0));
      // GetMotorZ()->GetController()->SendSetCommand(
      //     GetMotorZ()->GetIndex(), CMD_MOTOR_VOLTAGE, static_cast<int16_t>(0));
    }
  }

  // prev_update_time_ = now;
}
