#pragma once

struct FullAngle {
  int32_t rotation_count;
  uint16_t reminder;
};

#define NOT_SET -12345.0
#define _isset(a) ((a) != (NOT_SET))

#define MAX_VOLTAGE 28

using Velocity = int16_t;
using Angle = uint16_t;
using Voltage = int16_t;

inline Voltage VoltsToVoltage(uint16_t v) {
  return static_cast<int16_t>((v * 512));
}
