#include <stdint.h>

#ifndef BLDCDriver3PWM_h
#define BLDCDriver3PWM_h

// #include "../common/foc_utils.h"
// #include "../common/time_utils.h"
// #include "../common/defaults.h"
#include "hardware_api.h"
#include "types.h"

/**
 3 pwm bldc driver class
*/
class BLDCDriver3PWM {
 public:
  /**
    BLDCDriver class constructor
    @param phA A phase pwm pin
    @param phB B phase pwm pin
    @param phC C phase pwm pin
    @param en1 enable pin (optional input)
    @param en2 enable pin (optional input)
    @param en3 enable pin (optional input)
  */
  BLDCDriver3PWM(int phA, int phB, int phC, int en1 = NOT_SET,
                 int en2 = NOT_SET, int en3 = NOT_SET);

  /**  Motor hardware init function */
  int init();
  /** Motor disable function */
  void disable();
  /** Motor enable function */
  void enable();

  // hardware variables
  int pwmA;         //!< phase A pwm pin number
  int pwmB;         //!< phase B pwm pin number
  int pwmC;         //!< phase C pwm pin number
  // int enableA_pin;  //!< enable pin number
  // int enableB_pin;  //!< enable pin number
  // int enableC_pin;  //!< enable pin number
  bool enable_active_high = true;

  long pwm_frequency;             //!< pwm frequency value in hertz
  uint16_t voltage_power_supply;  //!< power supply voltage
  uint16_t voltage_limit;         //!< limiting voltage set to the motor

  uint8_t dc_a;  //!< currently set duty cycle on phaseA
  uint8_t dc_b;  //!< currently set duty cycle on phaseB
  uint8_t dc_c;  //!< currently set duty cycle on phaseC

  /**
   * Set phase voltages to the harware
   *
   * @param Ua - phase A voltage
   * @param Ub - phase B voltage
   * @param Uc - phase C voltage
   */
  void setPwm(Voltage Ua, Voltage Ub, Voltage Uc);

  /**
   * Set phase voltages to the harware
   *
   * @param sc - phase A state : active / disabled ( high impedance )
   * @param sb - phase B state : active / disabled ( high impedance )
   * @param sa - phase C state : active / disabled ( high impedance )
   */
  //void setPhaseState(int sa, int sb, int sc);

 private:
};

#endif
