#ifndef BLDCMotor_h
#define BLDCMotor_h

// #include "Arduino.h"
// #include "BLDCDriver3PWM.h"
#include "custom_magnetic_sensor_i2c.h"
// #include "common/configuration.h"
// #include "common/defaults.h"
// #include "common/foc_utils.h"
// #include "common/time_utils.h"
#include "types.h"

enum class MotionControlType {
  voltage,  //!< Torque control
  // velocity,  //!< Velocity motion control
  // angle,     //!< Position/angle motion control
  // velocity_openloop,
  // angle_openloop,
  direct_phase
};

/**
 BLDC motor class
*/
class BLDCMotor {
 public:
  /**
   BLDCMotor class constructor
   @param pp pole pairs number
   @param R  motor phase resistance
   */

  /**
    BLDCDriver class constructor
    @param phA A phase pwm pin
    @param phB B phase pwm pin
    @param phC C phase pwm pin
    @param en1 enable pin (optional input)
    @param en2 enable pin (optional input)
    @param en3 enable pin (optional input)
  */
  BLDCMotor(CustomMagneticSensorI2C* sensor, int phA, int phB, int phC,
            uint8_t pp, int8_t sensor_direction);

  /**  Motor hardware init function */
  int init();
  /** Motor disable function */
  void disable();
  /** Motor enable function */
  void enable();

  void linkSensor(CustomMagneticSensorI2C* sensor);

  // /**
  //  * Function initializing FOC algorithm
  //  * and aligning sensor's and motors' zero position
  //  */
  // int initFOC( float zero_electric_offset = NOT_SET , Direction
  // sensor_direction = Direction::CW) override;
  /**
   * Function running FOC algorithm in real-time
   * it calculates the gets motor angle and sets the appropriate voltages
   * to the phase pwm signals
   * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
   */
  void loopFOC();

  // /**
  //  * Function executing the control loops set by the controller parameter of
  //  the BLDCMotor.
  //  *
  //  * @param target  Either voltage, angle or velocity based on the
  //  motor.controller
  //  *                If it is not set the motor will use the target set in its
  //  variable motor.target
  //  *
  //  * This function doesn't need to be run upon each loop execution - depends
  //  of the use case
  //  */
  // void move(float target = NOT_SET) override;

  // hardware variables
  uint8_t pwmA;         //!< phase A pwm pin number
  uint8_t pwmB;         //!< phase B pwm pin number
  uint8_t pwmC;         //!< phase C pwm pin number
  uint8_t enableA_pin;  //!< enable pin number
  uint8_t enableB_pin;  //!< enable pin number
  uint8_t enableC_pin;  //!< enable pin number
  bool enable_active_high = true;

  int32_t Ua, Ub, Uc;  //!< Current phase voltages Ua,Ub and Uc set to motor
  // float Ualpha, Ubeta;  //!< Phase voltages U alpha and U beta used for
  // inverse
  //!< Park and Clarke transform

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
  void setPhaseState(int sa, int sb, int sc);

 private:
  // FOC methods
  /**
   * Method using FOC to set Uq to the motor at the optimal angle
   * Heart of the FOC algorithm
   *
   * @param Uq Current voltage in q axis to set to the motor
   * @param Ud Current voltage in d axis to set to the motor
   * @param angle_el current electrical angle of the motor
   */
  void setPhaseVoltage(Voltage U, Angle angle_el);
  void setPhaseVoltageSin(Voltage U, Angle angle_el);
  void setPhaseVoltageSin2(Voltage U, Angle angle_el);
  // Open loop motion control
  /**
   * Function (iterative) generating open loop movement for target velocity
   * it uses voltage_limit variable
   *
   * @param target_velocity - [-32767 - 32767]
   */
  uint16_t velocityOpenloop(Velocity target_velocity);
  /**
   * Function (iterative) generating open loop movement towards the target
   * angle it uses voltage_limit and velocity_limit variables
   *
   * @param target_angle - rad
   */
  uint16_t angleOpenloop(uint16_t target_angle);
  // open loop variables
  long open_loop_timestamp;

 public:
  Angle electricalAngle();

  // state variables
  // float target; //!< current target value - depends of the controller
  // FullAngle shaft_angle;   //!< current motor angle
  Angle electrical_angle;  //!< current electrical angle
  Voltage voltage = 0;     //!< current q voltage set to the motor
  int16_t phase = 0;       // for setting the phase directly

  uint8_t pole_pairs;  //!< motor pole pairs number

  Voltage voltage_limit =
      12 * (1 << 9) - 1;  //!< Voltage limitting variable - global limit
  Voltage voltage_power_supply = 12 * (1 << 9) - 1;
  long pwm_frequency;  //!< pwm frequency value in hertz
  int8_t enabled = 0;  //!< enabled or disabled motor flag

  int8_t modulation_centered = 1;  //!< flag (1) centered modulation around
                                   //!< driver limit /2  or  (0) pulled to 0

  // configuration structures
  MotionControlType
      controller;  //!< parameter determining the control loop to be used

  Angle zero_electric_angle =
      0;                        //!< absolute zero electric angle - if available
  int8_t sensor_direction = 1;  //!< if sensor_direction == Direction::CCW
                                //!< then direction will be flipped to CW

  uint8_t dc_a;
  uint8_t dc_b;
  uint8_t dc_c;

  CustomMagneticSensorI2C* sensor = nullptr;
};

#endif
