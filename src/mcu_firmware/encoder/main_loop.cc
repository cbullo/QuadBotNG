#include "SimpleFOC.h"
#include "base/custom_magnetic_sensor_i2c.h"

// Things that need to be guaranteed to get called. These are guarded with WDT
void Critical() {}

CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

unsigned long next_sensor_read;
void Initialize() {
  sensor.activate();
  sensor.init();
  _delay(1000);
  Serial.println(F("encoder_angle,encoder_velocity"));
  next_sensor_read = millis();
}

#define SENSOR_READ_PERIOD 100;  // ms
void Tick() {
  sensor.update();
  if (millis() >= next_sensor_read) {
    Serial.print(sensor.getAngle());
    Serial.print("\t");
    Serial.println(sensor.getVelocity());
    next_sensor_read += SENSOR_READ_PERIOD;
  }
}
