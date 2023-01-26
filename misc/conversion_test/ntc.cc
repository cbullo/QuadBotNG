#define RT0 47000.f  // Ω
#define B 3950.f     // K
//--------------------------------------

#define VCC 5.f    // Supply voltage
#define R 56000.f  // R=56KΩ

#define T0 (25.f + 273.15f)

#include <math.h>

#include <iostream>

uint8_t temp_lut[] = {10, 10, 11, 12, 12, 13, 14, 14, 15, 16, 16, 17, 18, 18,
                      19, 20, 20, 21, 22, 22, 23, 24, 25, 25, 26, 27, 27, 28,
                      29, 30, 30, 31, 32, 33, 34, 34, 35, 36, 37, 38, 39, 39,
                      40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54,
                      56, 57, 59, 60, 62, 63, 65, 67, 69, 71, 73, 76, 78};
uint8_t lut_offset = 47;

inline uint8_t ComputeTemperatureInt(uint16_t analog_reading) {
  analog_reading >>= 3;
  if (analog_reading < lut_offset) {
    return 0;
  }
  if (analog_reading > 14 * 5 - 2 + lut_offset) {
    return 255;
  }
  uint8_t temp = temp_lut[analog_reading - lut_offset];
  return temp;
}

inline float ComputeTemperature(int analog_reading) {
  auto v_reading =
      (5.f / 1023.f) * (1024 - analog_reading);  // Conversion to voltage
  auto v_offset = VCC - v_reading;
  auto resistance = v_reading / (v_offset / R);  // Resistance of RT

  auto ln = logf(resistance / RT0);
  auto temp = (1.f / ((ln / B) + (1.f / T0)));  // Temperature from thermistor
  temp -= 273.15;                               // Conversion to Celsius
  return temp;
}

int main() {
  for (int i = 0; i < 1024; i++) {
    std::cout << i << " " << (int)std::round(ComputeTemperature(i)) << " "
              << (int)ComputeTemperatureInt(i) << std::endl;
  }
  return 0;
}