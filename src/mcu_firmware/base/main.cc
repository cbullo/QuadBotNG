
//#include "../../custom_magnetic_sensor_i2c.h"
#include <avr/wdt.h>

#include "Arduino.h"
#include "SimpleFOC.h"
#include "Wire.h"
#include "analog_reader.h"
#include "temperature.h"

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

void Initialize();
void Tick();
void Critical();

void MainInitialize() {
  wdt_disable();
  Serial.begin(115200);
  // while (!Serial)
  //  ;

  // configure i2C
  Wire.setClock(400000);

  analog_reader.StartConversion(2, 0);
  Serial.println(F("Done setup"));
  wdt_enable(WDTO_1S);
  next_temperature_read = millis();
  Initialize();
}

void CheckTemperature(bool first) {
  auto reading = analog_reader.GetADCReading(first ? 0 : 1);
  // Serial.print(F("READING: "));

  auto temperature = ComputeTemperature(reading);
  if (temperature > 50) {
    Serial.print(F("temperature: "));
    Serial.println(temperature);
    Serial.println(F("OVERHEAT DETECTED! RESETTING!"));
    _delay(100000);  // This will trigger WDT reset
  }
  analog_reader.StartConversion(first ? 2 : 3, first ? 1 : 0);
}

#define TEMP_CHECK_PERIOD 1000;  // ms
void MainCritical() {
  if (millis() >= next_temperature_read) {
    // Add all safety/hardware failure critical functions here
    CheckTemperature(next_temperature_read % 2000 < 1000);
    next_temperature_read += TEMP_CHECK_PERIOD;
  }
  Critical();

  // This should be the only place where WDT is reset!
  wdt_reset();
}

int main() {
  init();

  MainInitialize();
  while (true) {
    Tick();
    MainCritical();
  }
  return 0;
}