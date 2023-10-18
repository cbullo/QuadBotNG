#pragma once

struct PIDParams {
  float p;
  float d;
  float i;
};

struct PIDState {
  float prev_error = 0.f;
  float i_sum = 0.f;
  float last_target = 0.f;
  float last_reset = 0.f;
  bool last_target_set = false;

  float approx_running_mean = 0.f;
  float approx_running_variance = 0.f;

  float time_since_value_stable = 0.f;
  float time_since_target_changed = 0.f;

  float variance = 0.f;

  void Reset() {
    prev_error = 0.f;
    i_sum = 0.f;
    last_target = 0.f;
    last_reset = 0.f;
    last_target_set = false;

    approx_running_mean = 0.f;
    approx_running_variance = 0.f;

    time_since_value_stable = 0.f;
    time_since_target_changed = 0.f;
  
    variance = 0.f;
  }
};

inline void UpdatePIDStatistics(PIDState& state, float value, float dt,
                                bool target_changed) {
  if (target_changed) {
    state.time_since_value_stable = 0.f;
    state.time_since_target_changed = 0.f;
    state.approx_running_mean = 0.f;
    state.approx_running_variance = 0.f;
    //std::cout << "Target changed" << std::endl;
    return;
  }

  state.approx_running_mean = state.approx_running_mean * (1.0f - dt) + value * dt;
  state.variance =
      (value - state.approx_running_mean) * (value - state.approx_running_mean);
  state.approx_running_variance =
      state.approx_running_variance * (1.0f - dt) + state.variance * dt;

  //if (state.time_since_target_changed > 1.f) {
    if (state.variance > 0.0001f) {
      state.time_since_value_stable = 0.f;
    } else {
      state.time_since_value_stable += dt;
    }
  //}

  state.time_since_target_changed += dt;
}

inline void DebugPIDControl(PIDState& state, float target, float value, float variance,
                            float p_term, float d_term, float i_term,
                            float tau) {
  std::cout << " T: " << target;
  std::cout << " V: " << value;
  std::cout << " Var: " << variance;
  std::cout << " P: " << p_term << " D: " << d_term << " I: " << i_term;
  std::cout << " Isum: " << state.i_sum;
  std::cout << " Tau: " << tau;
  std::cout << std::endl;
}

inline float UpdatePIDControl(PIDState& state, const PIDParams& params,
                              float dt, float target, float value, bool debug) {
  double error = target - value;  // ClosestAngle(theta, state_.theta_setpoint);
  if (error == 0.0 || error * state.prev_error < 0.0) {
    state.i_sum = 0.0;
  }
  state.i_sum = state.i_sum + dt * 0.5 * (error + state.prev_error);
  double p_term = params.p * error;
  double d_term = -params.d * (error - state.prev_error) / dt;
  double i_term = params.i * state.i_sum;
  double tau = p_term + d_term + i_term;
  state.prev_error = error;
  UpdatePIDStatistics(state, value, dt,
                      !state.last_target_set || fabsf(state.last_reset - target) > 0.1);
  if (fabsf(state.last_reset - target) > 0.1) {
    state.last_reset = target;
  }
  state.last_target = target;
  state.last_target_set = true;

  if (debug) {
    DebugPIDControl(state, target, value, state.variance, p_term, d_term, i_term, tau);
  }
  return tau;
}
