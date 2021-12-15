#pragma once

#include <math.h>

#define RT0 47000.f  // Ω
#define B 3950.f     // K
//--------------------------------------

#define VCC 5.f    // Supply voltage
#define R 56000.f  // R=56KΩ

#define T0 (25.f + 273.15f)

inline float ComputeTemperature(int analog_reading) {
  auto v_reading = (5.f / 1023.f) * (1024 - analog_reading);  // Conversion to voltage
  auto v_offset = VCC - v_reading;
  auto resistance = v_reading / (v_offset / R);  // Resistance of RT

  auto ln = logf(resistance / RT0);
  auto temp = (1.f / ((ln / B) + (1.f / T0)));  // Temperature from thermistor 
  temp -= 273.15;                             // Conversion to Celsius
  return temp;
}