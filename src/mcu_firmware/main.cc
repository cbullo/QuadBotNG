#include "Arduino.h"
#include "SimpleFOC.h"
#include "custom_magnetic_sensor_i2c.h"

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t
// _angle_register_msb)
//  chip_address  I2C chip address
//  bit_resolution  resolution of the sensor
//  angle_register_msb  angle read register msb
//  bits_used_msb  number of used bits in msb register
//
// make sure to read the chip address and the chip angle register msb value from
// the datasheet also in most cases you will need external pull-ups on SDA and
// SCL lines!!!!!
//
// For AS5058B
// MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// Example of AS5600 configuration
CustomMagneticSensorI2C sensor1 = CustomMagneticSensorI2C(AS5600_I2C, A0, A1);
CustomMagneticSensorI2C sensor2 = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

void setup() {
  // monitoring port
  Serial.begin(57600);

  // configure i2C
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor1.activate();
  sensor1.init();

  Serial.println("Sensor 1 ready");
  _delay(1000);

  sensor2.activate();
  sensor2.init();

  Serial.println("Sensor 2 ready");
  _delay(1000);
}

void loop() {
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor1.getAngle());
  Serial.print("\t");
  Serial.print(sensor1.getVelocity());
  Serial.print("\t");
  Serial.print(sensor2.getAngle());
  Serial.print("\t");
  Serial.println(sensor2.getVelocity());
}