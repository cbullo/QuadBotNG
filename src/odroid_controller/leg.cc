#include "leg.h"

#include <chrono>

#include "controller_board.h"
#include "motor.h"

Leg::Leg(Motor* m_f, Motor* m_r, Motor* m_z) {
  m_f_ = m_f;
  m_r_ = m_r;
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

void Leg::UpdateControl() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  // auto seconds = std::chrono::duration<double>(now).count();

  if (!time_initialized_) {
    prev_update_time_ = std::chrono::system_clock::now().time_since_epoch();
    time_initialized_ = true;
    return;
  }

  std::chrono::duration<double> elapsed = now - prev_update_time_;
  auto dt = elapsed.count();
  if (!active_command_->Process(*this, dt)) {
    // auto* cmd = GetQueue().GetNextCommand();
    // cmd->AllocateAndInheritState(queue_, *active_command_);
    // active_command_ = cmd;
  }

  prev_update_time_ = now;
}

bool ThetaGammaZCommand::Process(Leg& leg, double dt) {
  // double af = leg.GetAngleF();
  // double ar = leg.GetAngleR();
  double az = leg.GetAngleZ();

  double theta = leg.GetTheta();
  double gamma = leg.GetGamma();

  double theta_error = state_.theta_setpoint - theta;
  state_.i_theta_sum =
      state_.i_theta_sum + dt * 0.5 * (theta_error + state_.prev_theta_error);
  double p_term_theta = leg.theta_pd_config.p * (theta_error);
  double d_term_theta =
      -leg.theta_pd_config.d * (theta_error - state_.prev_theta_error) / dt;
  double i_term_theta = leg.theta_pd_config.i * state_.i_theta_sum;

  double gamma_error = state_.gamma_setpoint - gamma;
  state_.i_gamma_sum =
      state_.i_gamma_sum + dt * 0.5 * (gamma_error + state_.prev_gamma_error);

  double p_term_gamma = leg.gamma_pd_config.p * (gamma_error);
  double d_term_gamma =
      -leg.gamma_pd_config.d * (gamma_error - state_.prev_gamma_error) / dt;
  double i_term_gamma = leg.gamma_pd_config.i * state_.i_gamma_sum;

  double tau_theta = p_term_theta + d_term_theta + i_term_theta;
  double tau_gamma = p_term_gamma + d_term_gamma + i_term_gamma;

  double z_error = state_.z_setpoint - az;
  state_.i_z_sum = state_.i_z_sum + dt * 0.5 * (z_error + state_.prev_z_error);
  double p_term_z = leg.z_pd_config.p * (z_error);
  double d_term_z = -leg.z_pd_config.d * (z_error - state_.prev_z_error) / dt;
  double i_term_z = leg.z_pd_config.i * state_.i_z_sum;
  double tau_z = p_term_z + d_term_z + i_term_z;

  double m_f_control = tau_theta * 0.5f - tau_gamma * 0.5f;
  double m_r_control = tau_theta * 0.5f + tau_gamma * 0.5f;
  double m_z_control = tau_z /*- 0.75f * tau_gamma*/;

  state_.prev_gamma_error = gamma_error;
  state_.prev_theta_error = theta_error;
  state_.prev_z_error = z_error;

  m_f_control *= -leg.GetMotorF()->GetDirection();
  m_r_control *= -leg.GetMotorR()->GetDirection();
  m_z_control *= -leg.GetMotorZ()->GetDirection();

  int16_t m_f_control_i =
      std::clamp(m_f_control * Motor::kRadiansToAS5600, -255.0, 255.0);
  int16_t m_r_control_i =
      std::clamp(m_r_control * Motor::kRadiansToAS5600, -255.0, 255.0);
  int16_t m_z_control_i =
      std::clamp(m_z_control * Motor::kRadiansToAS5600, -255.0, 255.0);

  // std::cout << p_term_theta << " " << p_term_gamma << " " << m1_control_i <<
  // " "
  //           << m2_control_i << std::endl;

  return true;
}
