#include "Arduino.h"

// change this number when eeprom data structure has changed
#define EEPROM_VERSION 1

struct Config {
  uint8_t version = EEPROM_VERSION;
  uint8_t sensor0_linearization[16];
  uint8_t sensor1_linearization[16];
  uint8_t crc8 = 0;
};
extern config_t config;

inline void setDefaultParameters() { config = config_t(); }