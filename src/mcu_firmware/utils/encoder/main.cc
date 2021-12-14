
//#include "../../custom_magnetic_sensor_i2c.h"
#include "Arduino.h"
#include "../../analog_reader.h"
//#include "SimpleFOC.h"

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
// CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);
AnalogReader analog_reader = AnalogReader();

void setup() {
  delay(1);
  Serial.begin(115200);
  //while (!Serial)
  //  ;

  // configure i2C
  //Wire.setClock(400000);

  // sensor.activate();
  // sensor.init();
  //_delay(1000);
  analog_reader.StartConversion(2, 0);
  Serial.println(F("Done setup"));

}



void loop() {
  // display the angle and the angular velocity to the terminal
  // Serial.print(sensor.getAngle());
  // Serial.print("\t");
  // Serial.println(sensor.getVelocity());
  // if (!analog_reader.IsConverting()) {
  auto t_reading = analog_reader.GetADCReading(0);
  Serial.println(t_reading);
  analog_reader.StartConversion(2, 0);
  //}
}

int main() {
  init();
  
  setup();
  while (true) {
    loop();
  }
  return 0;
}