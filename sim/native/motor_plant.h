#pragma once
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <stdexcept>
namespace simulation {
struct MotorParameters {
  double voltage_limit = 28, torque_per_volt = 0.03, back_emf = 0.015;
  int voltage_sign = -1, zero_counts = 0;
  void Validate() const {
    if (!std::isfinite(voltage_limit) || !std::isfinite(torque_per_volt) || !std::isfinite(back_emf) ||
        voltage_limit <= 0 || torque_per_volt <= 0 || back_emf < 0 || std::abs(voltage_sign) != 1)
      throw std::invalid_argument("invalid motor approximation parameters");
  }
  double Torque(double voltage, double speed, double limit, bool enabled) const {
    if (!enabled) return 0; // disabled bridge coasts; plant friction remains
    return std::clamp(torque_per_volt * (voltage_sign * std::clamp(voltage,-voltage_limit,voltage_limit)
                       - back_emf * speed), -limit, limit);
  }
  uint16_t Encoder(double angle) const {
    const double counts = std::floor(angle * (4096.0 / (2 * std::acos(-1.0)))) + zero_counts;
    const auto wrapped = static_cast<int>(std::fmod(counts,4096));
    return (wrapped + 4096) % 4096;
  }
};

// Lumped motor/sensor temperature. Parameters are effective values pending
// measurements; no thermistor location, ADC filtering or winding/sensor lag is
// inferred from the CAD. Board resets never own this physical state.
struct ThermalParameters {
  double ambient_c = 20, initial_c = 20;
  double resistance_ohm = 0.5;
  double heat_capacity_j_per_k = 20;
  double thermal_resistance_k_per_w = 8;
  void Validate() const {
    if (!std::isfinite(ambient_c) || !std::isfinite(initial_c) ||
        ambient_c < -273.15 || initial_c < -273.15 ||
        !std::isfinite(resistance_ohm) || resistance_ohm <= 0 ||
        !std::isfinite(heat_capacity_j_per_k) || heat_capacity_j_per_k <= 0 ||
        !std::isfinite(thermal_resistance_k_per_w) || thermal_resistance_k_per_w <= 0)
      throw std::invalid_argument("invalid thermal parameters");
  }
};
class MotorTemperature {
 public:
  explicit MotorTemperature(ThermalParameters parameters = {}) : parameters_(parameters) {
    parameters_.Validate();
    Reset();
  }
  void Reset() { temperature_c_ = parameters_.initial_c; }
  double Celsius() const { return temperature_c_; }
  uint16_t Sensor() const {
    // Firmware transmits whole degrees C in uint16, from an 8-bit conversion.
    return static_cast<uint16_t>(std::clamp(std::round(temperature_c_),0.0,255.0));
  }
  void Step(double shaft_torque, double torque_per_volt, double dt) {
    if (!std::isfinite(shaft_torque) || !std::isfinite(torque_per_volt) || torque_per_volt <= 0 ||
        !std::isfinite(dt) || dt < 0)
      throw std::invalid_argument("invalid thermal step");
    // The existing static plant has gain = Kt/R. Infer Kt from that gain and
    // effective resistance, using the actual saturated torque (zero if off).
    const double current = shaft_torque / (torque_per_volt * parameters_.resistance_ohm);
    const double loss = current * current * parameters_.resistance_ohm;
    const double equilibrium = parameters_.ambient_c + loss * parameters_.thermal_resistance_k_per_w;
    const double fraction = -std::expm1(-dt / parameters_.heat_capacity_j_per_k /
                                      parameters_.thermal_resistance_k_per_w);
    // Exact update for constant loss over one physics step; no Euler overshoot.
    const double next = temperature_c_ + (equilibrium - temperature_c_) * fraction;
    if (!std::isfinite(next)) throw std::runtime_error("non-finite motor temperature");
    temperature_c_ = next;
  }
 private:
  ThermalParameters parameters_;
  double temperature_c_;
};
}  // namespace simulation
