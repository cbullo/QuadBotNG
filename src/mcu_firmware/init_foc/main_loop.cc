/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */

#include <avr/wdt.h>

#include "Arduino.h"
#include "SimpleFOC.h"
#include "base/custom_magnetic_sensor_i2c.h"

#define POLE_PAIR_NUMBER 7
#define PHASE_RESISTANCE 5 //ohm

unsigned long next_sensor_read;

CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

// BLDC motor & driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 3, 6);
BLDCMotor motor = BLDCMotor(POLE_PAIR_NUMBER, PHASE_RESISTANCE);

void Critical() {};

void Initialize() {
  // initialise magnetic sensor hardware
  sensor.activate();
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  motor.current_limit = 0.5;
  motor.velocity_limit = 20;

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.05;

  // angle P controller
  motor.P_angle.P = 15;
  motor.P_angle.I = 100;
  motor.P_angle.D = 0.8;

  // maximal velocity of the position control
  motor.velocity_limit = 20;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();

  // align sensor and start FOC
  // motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
}

bool initialized = false;

int repeat_foc_init = 5;

unsigned long prev_us;
void Tick() {
  unsigned long now_us = _micros();
  // Serial.println(now_us - prev_us);
  prev_us = now_us;

  while (repeat_foc_init > 0) {
    wdt_enable(WDTO_8S);
    motor.zero_electric_angle = NOT_SET;
    motor.sensor_direction = NOT_SET;
    Serial.println(F("Initializing FOC."));
    motor.initFOC();
    initialized = true;
    wdt_enable(WDTO_1S);
    repeat_foc_init -= 1;
  }
  

  // motor.loopFOC();
  // motor.move(10);

  // motor.monitor();
}
