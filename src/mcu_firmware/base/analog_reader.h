#pragma once

#include "Arduino.h"

class AnalogReader {
 public:
  AnalogReader() {
  }

  bool StartConversion(byte adc_pin, byte index);
  int GetADCReading(byte index);
  //bool IsConverting() { return adc_working_ != -1; }

  //volatile static int adc_reading_;
  //volatile static signed char adc_working_;
};