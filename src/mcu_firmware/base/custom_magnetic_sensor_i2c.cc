#include "custom_magnetic_sensor_i2c.h"

void CustomMagneticSensorI2C::Activate() {
  pinMode(this_sda_pin_, INPUT);
  pinMode(other_sda_pin_, OUTPUT);
  digitalWrite(other_sda_pin_, HIGH);
}

float CustomMagneticSensorI2C::getSensorAngle() {
  Activate();
  float angle = MagneticSensorI2C::getSensorAngle();
  uint16_t angle_int = round(angle * kRadToEnc);
  int16_t correction = linearization_.Value(angle_int);
  uint16_t angle_corrected = (angle_int + correction) & (0xFFF);
  Serial.print(angle_int);
  Serial.print(" ");
  Serial.println(angle_corrected);
  return angle_corrected * kEncToRad;
}