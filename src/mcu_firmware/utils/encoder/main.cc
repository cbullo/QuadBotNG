
//#include "../../custom_magnetic_sensor_i2c.h"
#include <avr/wdt.h>

#include "../../analog_reader.h"
#include "../../temperature.h"
#include "Arduino.h"
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

unsigned long next_temperature_read;

void Setup() {
  wdt_disable();
  Serial.begin(115200);
  // while (!Serial)
  //  ;

  // configure i2C
  // Wire.setClock(400000);

  // sensor.activate();
  // sensor.init();
  //_delay(1000);
  analog_reader.StartConversion(2, 0);
  Serial.println(F("Done setup"));
  wdt_enable(WDTO_1S);
  next_temperature_read = millis();
}

void CheckTemperature() {
  auto reading = analog_reader.GetADCReading(0);
  Serial.print(F("READING: "));
  Serial.println(reading);
  Serial.print(F("TEMPERATURE: "));
  Serial.println(ComputeTemperature(reading));
  analog_reader.StartConversion(2, 0);
  
}

#define TEMP_CHECK_PERIOD 1000; //ms
void Critical() {
  if (millis() >= next_temperature_read) {
  // Add all safety/hardware failure critical functions here
    CheckTemperature();
    next_temperature_read += TEMP_CHECK_PERIOD;
  }

  // This should be the only place where WDT is reset!
  wdt_reset();
}

void Loop() {
  // display the angle and the angular velocity to the terminal
  // Serial.print(sensor.getAngle());
  // Serial.print("\t");
  // Serial.println(sensor.getVelocity());
  // if (!analog_reader.IsConverting()) {
  //}

  Critical();
}

int main() {
  init();

  Setup();
  while (true) {
    Loop();
  }
  return 0;
}