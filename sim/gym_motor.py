"""Vectorized training plant matching native/motor_plant.h, without board I/O."""

import numpy as np
import yaml

from sim.robot_model import SIM_DIR


class VoltageMotorPlant:
    def __init__(self, torque_limits, config_path=None):
        settings = yaml.safe_load((config_path or (SIM_DIR / "motor_model.yaml")).read_text())
        if settings["model"] != "sinusoidal_voltage":
            raise ValueError("expected sinusoidal_voltage motor model")
        self.voltage_limit = float(settings["voltage_limit"])
        self.resistance = float(settings["line_to_line_resistance_ohm"]) / 2
        self.ke = float(settings["phase_back_emf_volts_per_rad_s"])
        self.sign = float(settings["voltage_to_encoder_sign"])
        self.kp = float(settings["stance"]["p_volts_per_rad"])
        self.kd = float(settings["stance"]["d_volts_per_rad_s"])
        thermal = settings["thermal"]
        self.ambient = float(thermal["ambient_c"])
        self.initial = float(thermal["initial_c"])
        self.capacity = float(thermal["heat_capacity_j_per_k"])
        self.thermal_resistance = float(thermal["thermal_resistance_k_per_w"])
        self.limits = np.asarray(torque_limits, dtype=float).copy()
        positive = [self.voltage_limit, self.resistance, self.ke, self.kp,
                    self.capacity, self.thermal_resistance, *self.limits]
        if not np.isfinite(positive).all() or min(positive) <= 0:
            raise ValueError("invalid motor/thermal parameters")
        if (self.sign not in (-1, 1) or not np.isfinite(self.kd) or self.kd < 0
                or not np.isfinite([self.ambient, self.initial]).all()
                or min(self.ambient, self.initial) < -273.15):
            raise ValueError("invalid motor sign, damping or temperature")
        self.reset()

    def reset(self):
        self.temperature = np.full_like(self.limits, self.initial)

    def voltage(self, target, position, velocity):
        return np.clip(self.sign * (self.kp * (target - position) - self.kd * velocity),
                       -self.voltage_limit, self.voltage_limit)

    def torque(self, voltage, velocity):
        phase_voltage = .5 * self.sign * np.clip(voltage, -self.voltage_limit, self.voltage_limit)
        current = (phase_voltage - self.ke * velocity) / self.resistance
        return np.clip(1.5 * self.ke * current, -self.limits, self.limits)

    def copper_loss(self, torque):
        """Per-motor copper heating in watts, from applied shaft torque."""
        return 1.5 * self.resistance * (torque / (1.5 * self.ke))**2

    def heat(self, torque, dt):
        loss = self.copper_loss(torque)
        equilibrium = self.ambient + loss * self.thermal_resistance
        fraction = -np.expm1(-dt / self.capacity / self.thermal_resistance)
        self.temperature += (equilibrium - self.temperature) * fraction
