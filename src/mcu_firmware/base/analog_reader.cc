#include "analog_reader.h"

#include "debug_led.h"

//#include "Arduino.h"

//#include <avr/interrupt.h>

volatile int adc_reading_[2]{-1, -1};
volatile signed char adc_working_{-1};

// ADC complete ISR
ISR(ADC_vect) {
  byte low, high;

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low = ADCL;
  high = ADCH;

  adc_reading_[adc_working_] = (high << 8) | low;
  adc_working_ = -1;
}

int AnalogReader::GetADCReading(byte index) {
  // if (bit_is_clear(ADCSRA, ADSC)) {
  //   int value = ADC;
  //   adc_reading_[adc_working_] = value;
  //   adc_working_ = -1;
  // }
  return adc_reading_[index];
}

bool AnalogReader::StartConversion(byte adc_pin, byte index) {
  // if we aren't taking a reading, start a new one
  if (adc_working_ == -1) {
    adc_working_ = index;
    ADCSRA = bit(ADEN);                              // turn ADC on
    ADCSRA |= bit(ADPS0) | bit(ADPS1) | bit(ADPS2);  // Prescaler of 128
    ADMUX = bit(REFS0) | (adc_pin & 0x07);  // select AVcc and select input port
    ADCSRA |= bit(ADIE);
    ADCSRA |= bit(ADSC);  // start the conversion
    return true;
  }
  return false;
}
