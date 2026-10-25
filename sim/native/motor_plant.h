#pragma once
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <stdexcept>
namespace simulation {
struct MotorParameters {
  double voltage_limit = 28, resistance_line_to_line = 10, phase_back_emf = 0.0700475;
  int voltage_sign = -1, zero_counts = 0;
  void Validate() const {
    if (!std::isfinite(voltage_limit) || !std::isfinite(resistance_line_to_line) || !std::isfinite(phase_back_emf) ||
        voltage_limit <= 0 || resistance_line_to_line <= 0 || phase_back_emf <= 0 || std::abs(voltage_sign) != 1)
      throw std::invalid_argument("invalid motor approximation parameters");
  }
  double PhaseResistance() const { return resistance_line_to_line / 2; }
  // Peak phase / amplitude-invariant dq convention, mechanical rotor rad/s.
  double TorqueConstant() const { return 1.5 * phase_back_emf; }
  double Torque(double voltage, double speed, double limit, bool enabled) const {
    if (!enabled) return 0; // disabled bridge coasts; plant friction remains
    // Active AVR setPhaseVoltageSin2 maps U to duty ~|U|/(2*Vbus)*(1+sin).
    // Its phase-neutral sinusoid amplitude is U/2, not U. Keep the common
    // mode out of the winding voltage; electrical calibration stays on boards.
    const double phase_voltage = 0.5 * voltage_sign * std::clamp(voltage,-voltage_limit,voltage_limit);
    const double current = (phase_voltage - phase_back_emf * speed) / PhaseResistance();
    return std::clamp(TorqueConstant() * current, -limit, limit);
  }
  double CopperLoss(double applied_torque) const {
    const double current = applied_torque / TorqueConstant();
    return 1.5 * PhaseResistance() * current * current;
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
  double heat_capacity_j_per_k = 20;
  double thermal_resistance_k_per_w = 8;
  void Validate() const {
    if (!std::isfinite(ambient_c) || !std::isfinite(initial_c) ||
        ambient_c < -273.15 || initial_c < -273.15 ||
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
  void Step(double copper_loss_w, double dt) {
    if (!std::isfinite(copper_loss_w) || copper_loss_w < 0 ||
        !std::isfinite(dt) || dt < 0)
      throw std::invalid_argument("invalid thermal step");
    const double equilibrium = parameters_.ambient_c + copper_loss_w * parameters_.thermal_resistance_k_per_w;
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
