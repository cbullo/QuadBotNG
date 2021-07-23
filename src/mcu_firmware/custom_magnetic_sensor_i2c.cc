#include "custom_magnetic_sensor_i2c.h"

void CustomMagneticSensorI2C::activate() {
  pinMode(this_sda_pin_, INPUT);
  pinMode(other_sda_pin_, OUTPUT);
  digitalWrite(other_sda_pin_, HIGH);
}

float CustomMagneticSensorI2C::getAngle() {
  activate();
  return MagneticSensorI2C::getAngle();
}
/** get current angular velocity (rad/s)*/
float CustomMagneticSensorI2C::getVelocity() {
  activate();
  return MagneticSensorI2C::getVelocity();
}