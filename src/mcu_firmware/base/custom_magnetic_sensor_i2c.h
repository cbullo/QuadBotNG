#pragma once

#include "piecewise_linear.h"
#include "sensors/MagneticSensorI2C.h"

const static float kRadToEnc = 4096.f / _2PI;
const static float kEncToRad = _2PI / 4096.f;

class CustomMagneticSensorI2C : public MagneticSensorI2C {
 public:
  CustomMagneticSensorI2C(uint8_t chip_address, int bit_resolution,
                          uint8_t angle_register_msb, int msb_bits_used,
                          int this_sda_pin, int other_sda_pin)
      : MagneticSensorI2C(chip_address, bit_resolution, angle_register_msb,
                          msb_bits_used),
        this_sda_pin_(this_sda_pin),
        other_sda_pin_(other_sda_pin) {}

  CustomMagneticSensorI2C(MagneticSensorI2CConfig_s config, int this_sda_pin,
                          int other_sda_pin)
      : MagneticSensorI2C(config),
        this_sda_pin_(this_sda_pin),
        other_sda_pin_(other_sda_pin) {}

  void Activate();

  /** get current angle (rad) */
  float getSensorAngle() override;

  PiecewiseLinear<12, 4> linearization_;

 private:
  int this_sda_pin_;
  int other_sda_pin_;
};
