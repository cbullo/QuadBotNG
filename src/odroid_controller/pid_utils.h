struct PIDParams {
  float p;
  float d;
  float i;
};

struct PIDState {
  float prev_error;
  float i_sum;
};

float UpdatePIDControl(PIDState& state, const PIDParams& params, float dt,
                       float target, float value) {
  double error = value - target;  // ClosestAngle(theta, state_.theta_setpoint);
  if (error == 0.0 || error * state.prev_error < 0.0) {
    state.i_sum = 0.0;
  }
  state.i_sum = state.i_sum + dt * 0.5 * (error + state.prev_error);
  double p_term = params.p * error;
  double d_term = -params.d * (error - state.prev_error) / dt;
  double i_term = params.i * state.i_sum;
  double tau = p_term + d_term + i_term;
  return tau;
}