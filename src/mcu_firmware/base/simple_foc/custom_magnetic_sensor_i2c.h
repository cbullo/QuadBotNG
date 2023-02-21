#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "piecewise_linear.h"
#include "types.h"
// #include "sensors/MagneticSensorI2C.h"

// const static float kRadToEnc = 4096.f / _2PI;
// const static float kEncToRad = _2PI / 4096.f;

struct MagneticSensorI2CConfig_s {
  int chip_address;
  int bit_resolution;
  int angle_register;
  int data_start_bit;
};

class CustomMagneticSensorI2C {
 public:
  CustomMagneticSensorI2C(MagneticSensorI2CConfig_s config, int this_sda_pin,
                          int other_sda_pin);
  CustomMagneticSensorI2C(uint8_t chip_address, int bit_resolution,
                          uint8_t angle_register_msb, int msb_bits_used,
                          int this_sda_pin, int other_sda_pin);

  void Activate();
  /* Get mechanical shaft angle in the range 0 to 2PI. This value will be as
   * precise as possible with the hardware. Base implementation uses the values
   * returned by update() so that the same values are returned until update() is
   * called again.
   */
  Angle getMechanicalAngle() const;

  /**
   * Updates the sensor values by reading the hardware sensor.
   * Some implementations may work with interrupts, and not need this.
   * The base implementation calls getSensorAngle(), and updates internal
   * fields for angle, timestamp and full rotations.
   * This method must be called frequently enough to guarantee that full
   * rotations are not "missed" due to infrequent polling.
   * Override in subclasses if alternative behaviours are required for your
   * sensor hardware.
   */
  void update();

  long getLastAngleReadingTime() const { return last_angle_reading_time_; };

  PiecewiseLinear<12, 4> linearization_;

 protected:
  /**
   * Get current shaft angle from the sensor hardware, and
   * return it as a float in radians, in the range 0 to 2PI.
   *
   * This method is pure virtual and must be implemented in subclasses.
   * Calling this method directly does not update the base-class internal
   * fields. Use update() when calling from outside code.
   */
  Angle getSensorAngle() const { getMechanicalAngle(); }

 public:
  /**
   * Call Sensor::init() from your sensor subclass's init method if you want
   * smoother startup The base class init() method calls getSensorAngle()
   * several times to initialize the internal fields to current values, ensuring
   * there is no discontinuity ("jump from zero") during the first calls to
   * sensor.getAngle() and sensor.getVelocity()
   */
  void init(TwoWire* _wire = &Wire);

 private:
  Angle getRawCount();
  int read(uint8_t angle_reg_msb);
  int checkBus(byte sda_pin, byte scl_pin);

  uint16_t last_angle_reading_ = 0;
  long last_angle_reading_time_ = -1;

  uint8_t this_sda_pin_;
  uint8_t other_sda_pin_;

  uint16_t lsb_used;  //!< Number of bits used in LSB register
  uint8_t lsb_mask;
  uint8_t msb_mask;

  // I2C variables
  uint8_t angle_register_msb;  //!< I2C angle register to read
  uint8_t chip_address;        //!< I2C chip select pins

  /* the two wire instance for this sensor */
  TwoWire* wire;
};
