#include "Arduino.h"
#include "hardware_api.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328PB__)

// set pwm frequency to 32KHz
void _pinHighFrequency(const int pin) {
  //  High PWM frequency
  //  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
  if (pin == 5 || pin == 6) {
    TCCR0A = ((TCCR0A & 0b11111100) |
              0x01);  // configure the pwm phase-corrected mode
    TCCR0B = ((TCCR0B & 0b11110000) | 0x01);  // set prescaler to 1
  }
  if (pin == 9 || pin == 10)
    TCCR1B = ((TCCR1B & 0b11111000) | 0x01);  // set prescaler to 1
  if (pin == 3 || pin == 11)
    TCCR2B = ((TCCR2B & 0b11111000) | 0x01);  // set prescaler to 1
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// supports Arudino/ATmega328
void _configure3PWM(long pwm_frequency, const int pinA, const int pinB,
                    const int pinC) {
  //_UNUSED(pwm_frequency);
  //  High PWM frequency
  // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
  _pinHighFrequency(pinC);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _writeDutyCycle3PWM(uint8_t dc_a, uint8_t dc_b, uint8_t dc_c, int pinA,
                         int pinB, int pinC) {
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, dc_a);
  analogWrite(pinB, dc_b);
  analogWrite(pinC, dc_c);
}

// function configuring pair of high-low side pwm channels, 32khz frequency and
// center aligned pwm
int _configureComplementaryPair(int pinH, int pinL) {
  if ((pinH == 5 && pinL == 6) || (pinH == 6 && pinL == 5)) {
    // configure the pwm phase-corrected mode
    TCCR0A = ((TCCR0A & 0b11111100) | 0x01);
    // configure complementary pwm on low side
    if (pinH == 6)
      TCCR0A = 0b10110000 | (TCCR0A & 0b00001111);
    else
      TCCR0A = 0b11100000 | (TCCR0A & 0b00001111);
    // set prescaler to 1 - 32kHz freq
    TCCR0B = ((TCCR0B & 0b11110000) | 0x01);
  } else if ((pinH == 9 && pinL == 10) || (pinH == 10 && pinL == 9)) {
    // set prescaler to 1 - 32kHz freq
    TCCR1B = ((TCCR1B & 0b11111000) | 0x01);
    // configure complementary pwm on low side
    if (pinH == 9)
      TCCR1A = 0b10110000 | (TCCR1A & 0b00001111);
    else
      TCCR1A = 0b11100000 | (TCCR1A & 0b00001111);
  } else if ((pinH == 3 && pinL == 11) || (pinH == 11 && pinL == 3)) {
    // set prescaler to 1 - 32kHz freq
    TCCR2B = ((TCCR2B & 0b11111000) | 0x01);
    // configure complementary pwm on low side
    if (pinH == 11)
      TCCR2A = 0b10110000 | (TCCR2A & 0b00001111);
    else
      TCCR2A = 0b11100000 | (TCCR2A & 0b00001111);
  } else {
    return -1;
  }
  return 0;
}

// function setting the
// void _setPwmPair(int pinH, int pinL, float val, int dead_time) {
//   int pwm_h = _constrain(val - dead_time / 2, 0, 255);
//   int pwm_l = _constrain(val + dead_time / 2, 0, 255);

//   analogWrite(pinH, pwm_h);
//   if (pwm_l == 255 || pwm_l == 0)
//     digitalWrite(pinL, pwm_l ? LOW : HIGH);
//   else
//     analogWrite(pinL, pwm_l);
// }

#endif