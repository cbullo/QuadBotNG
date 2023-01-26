#include "BLDCDriver3PWM.h"

#include "Arduino.h"
#include "default_configuration.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2,
                               int en3) {
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  // enableA_pin = en1;
  // enableB_pin = en2;
  // enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = 0;
  pwm_frequency = NOT_SET;
}

// enable motor driver
void BLDCDriver3PWM::enable() {
  // enable_pin the driver - if enable_pin pin available
  // if (_isset(enableA_pin)) digitalWrite(enableA_pin, enable_active_high);
  // if (_isset(enableB_pin)) digitalWrite(enableB_pin, enable_active_high);
  // if (_isset(enableC_pin)) digitalWrite(enableC_pin, enable_active_high);
  // set zero to PWM
  setPwm(0, 0, 0);
}

// disable motor driver
void BLDCDriver3PWM::disable() {
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  // if (_isset(enableA_pin)) digitalWrite(enableA_pin, !enable_active_high);
  // if (_isset(enableB_pin)) digitalWrite(enableB_pin, !enable_active_high);
  // if (_isset(enableC_pin)) digitalWrite(enableC_pin, !enable_active_high);
}

// init hardware pins
int BLDCDriver3PWM::init() {
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  // if (_isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
  // if (_isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
  // if (_isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);

  // sanity check for the voltage limit configuration
  if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    voltage_limit = voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
  return 0;
}

// // Set voltage to the pwm pin
// void BLDCDriver3PWM::setPhaseState(int sa, int sb, int sc) {
//   // disable if needed
//   if (_isset(enableA_pin) && _isset(enableB_pin) && _isset(enableC_pin)) {
//     digitalWrite(enableA_pin, sa == _HIGH_IMPEDANCE ? LOW : HIGH);
//     digitalWrite(enableB_pin, sb == _HIGH_IMPEDANCE ? LOW : HIGH);
//     digitalWrite(enableC_pin, sc == _HIGH_IMPEDANCE ? LOW : HIGH);
//   }
// }

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(Voltage Ua, Voltage Ub, Voltage Uc) {
  // limit the voltage in driver
  Ua = max(0, min(Ua, voltage_limit));
  Ub = max(0, min(Ub, voltage_limit));
  Uc = max(0, min(Uc, voltage_limit));

  uint16_t Ua_u = Ua >> 2;
  uint16_t Ub_u = Ub >> 2;
  uint16_t Uc_u = Uc >> 2;

  dc_a = Ua / voltage_power_supply;
  dc_b = Ub / voltage_power_supply;
  dc_c = Uc / voltage_power_supply;

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);
}
