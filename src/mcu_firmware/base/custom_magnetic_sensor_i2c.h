#pragma once

#include "sensors/MagneticSensorI2C.h"

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

  void activate();

  /** get current angle (rad) */
  float getSensorAngle() override;

 private:
  int this_sda_pin_;
  int other_sda_pin_;
};
