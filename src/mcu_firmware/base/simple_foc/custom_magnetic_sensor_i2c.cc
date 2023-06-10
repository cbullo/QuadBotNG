#include "custom_magnetic_sensor_i2c.h"

#include "time_utils.h"

// inline Angle AS5600TicksToAngle(uint16_t encoder_angle) {
//   return encoder_angle << 4;
// }

void CustomMagneticSensorI2C::Activate() {
  pinMode(this_sda_pin_, INPUT);
  pinMode(other_sda_pin_, OUTPUT);
  digitalWrite(other_sda_pin_, HIGH);
}

// function reading the raw counter of the magnetic sensor
Angle CustomMagneticSensorI2C::getRawCount() {
  Activate();

  uint16_t raw_reading =
      (uint16_t)CustomMagneticSensorI2C::read(angle_register_msb);

  int16_t correction = linearization_.Value(raw_reading);
  uint16_t angle_corrected =
      static_cast<uint16_t>(raw_reading + correction) & (0xFFF);
  return angle_corrected;
}

void CustomMagneticSensorI2C::update() {
  uint16_t val = getRawCount();
  last_angle_reading_time_ = _micros();
  last_angle_reading_ = val;
}

Angle CustomMagneticSensorI2C::getMechanicalAngle() const {
  return last_angle_reading_;
}

// /** Typical configuration for the 12bit AMS AS5600 magnetic sensor over I2C
//  * interface */
// MagneticSensorI2CConfig_s AS5600_I2C = {.chip_address = 0x36,
//                                         .bit_resolution = 12,
//                                         .angle_register = 0x0C,
//                                         .data_start_bit = 11};

// /** Typical configuration for the 12bit AMS AS5048 magnetic sensor over I2C
//  * interface */
// MagneticSensorI2CConfig_s AS5048_I2C = {
//     .chip_address = 0x40,  // highly configurable.  if A1 and A2 are held
//     low,
//                            // this is probable value
//     .bit_resolution = 14,
//     .angle_register = 0xFE,
//     .data_start_bit = 15};

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t
// _angle_register_msb)
//  @param _chip_address  I2C chip address
//  @param _bit_resolution  bit resolution of the sensor
//  @param _angle_register_msb  angle read register
//  @param _bits_used_msb number of used bits in msb

CustomMagneticSensorI2C::CustomMagneticSensorI2C(
    MagneticSensorI2CConfig_s config, int this_sda_pin, int other_sda_pin) {
  this_sda_pin_ = this_sda_pin;
  other_sda_pin_ = other_sda_pin;

  chip_address = config.chip_address;

  // angle read register of the magnetic sensor
  angle_register_msb = config.angle_register;
  // register maximum value (counts per revolution)
  // cpr = pow(2, config.bit_resolution);

  int bits_used_msb = config.data_start_bit - 7;
  lsb_used = config.bit_resolution - bits_used_msb;
  // extraction masks
  lsb_mask = (uint8_t)((2 << lsb_used) - 1);
  msb_mask = (uint8_t)((2 << bits_used_msb) - 1);
  wire = &Wire;
}

CustomMagneticSensorI2C::CustomMagneticSensorI2C(
    uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb,
    int _bits_used_msb, int this_sda_pin, int other_sda_pin) {
  this_sda_pin_ = this_sda_pin;
  other_sda_pin_ = other_sda_pin;

  // chip I2C address
  chip_address = _chip_address;
  // angle read register of the magnetic sensor
  angle_register_msb = _angle_register_msb;
  // register maximum value (counts per revolution)
  // cpr = pow(2, _bit_resolution);

  // depending on the sensor architecture there are different combinations of
  // LSB and MSB register used bits
  // AS5600 uses 0..7 LSB and 8..11 MSB
  // AS5048 uses 0..5 LSB and 6..13 MSB
  // used bits in LSB
  lsb_used = _bit_resolution - _bits_used_msb;
  // extraction masks
  lsb_mask = (uint8_t)((2 << lsb_used) - 1);
  msb_mask = (uint8_t)((2 << _bits_used_msb) - 1);
  wire = &Wire;
}

void CustomMagneticSensorI2C::init(TwoWire* _wire) {
  wire = _wire;

  // I2C communication begin
  wire->begin();

  last_angle_reading_ = getRawCount();
}

// I2C functions
/*
 * Read a register from the sensor
 * Takes the address of the register as a uint8_t
 * Returns the value of the register
 */
int CustomMagneticSensorI2C::read(uint8_t angle_reg_msb) {
  // read the angle register first MSB then LSB
  byte readArray[2];
  uint16_t readValue = 0;
  // notify the device that is aboout to be read
  wire->beginTransmission(chip_address);
  wire->write(angle_reg_msb);
  wire->endTransmission(false);

  // read the data msb and lsb
  wire->requestFrom(chip_address, (uint8_t)2);
  for (byte i = 0; i < 2; i++) {
    readArray[i] = wire->read();
  }

  // depending on the sensor architecture there are different combinations of
  // LSB and MSB register used bits
  // AS5600 uses 0..7 LSB and 8..11 MSB
  // AS5048 uses 0..5 LSB and 6..13 MSB
  readValue = (readArray[1] & lsb_mask);
  readValue += ((readArray[0] & msb_mask) << lsb_used);
  return readValue;
}

/*
 * Checks whether other devices have locked the bus. Can clear SDA locks.
 * This should be called before sensor.init() on devices that suffer i2c slaves
 * locking sda e.g some stm32 boards with AS5600 chips Takes the sda_pin and
 * scl_pin Returns 0 for OK, 1 for other master and 2 for unfixable sda locked
 * LOW
 */
int CustomMagneticSensorI2C::checkBus(byte sda_pin, byte scl_pin) {
  pinMode(scl_pin, INPUT_PULLUP);
  pinMode(sda_pin, INPUT_PULLUP);
  delay(250);

  if (digitalRead(scl_pin) == LOW) {
    // Someone else has claimed master!");
    return 1;
  }

  if (digitalRead(sda_pin) == LOW) {
    // slave is communicating and awaiting clocks, we are blocked
    pinMode(scl_pin, OUTPUT);
    for (byte i = 0; i < 16; i++) {
      // toggle clock for 2 bytes of data
      digitalWrite(scl_pin, LOW);
      delayMicroseconds(20);
      digitalWrite(scl_pin, HIGH);
      delayMicroseconds(20);
    }
    pinMode(sda_pin, INPUT);
    delayMicroseconds(20);
    if (digitalRead(sda_pin) == LOW) {
      // SDA still blocked
      return 2;
    }
    _delay(1000);
  }
  // SDA is clear (HIGH)
  pinMode(sda_pin, INPUT);
  pinMode(scl_pin, INPUT);

  return 0;
}
