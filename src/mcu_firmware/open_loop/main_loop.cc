/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */

#include "Arduino.h"
#include "SimpleFOC.h"
#include "base/custom_magnetic_sensor_i2c.h"

void Critical() {}

#define POLE_PAIR_NUMBER 7
#define PHASE_RESISTANCE 5 //ohm

unsigned long next_sensor_read;

CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

// BLDC motor & driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 3, 6);
BLDCMotor motor = BLDCMotor(POLE_PAIR_NUMBER, PHASE_RESISTANCE);

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

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity_openloop;

  // initialize motor
  motor.init();

  Serial.println(F("Motor ready."));

  next_sensor_read = millis();
}

#define SENSOR_READ_PERIOD 100;  // ms
void Tick() {
  sensor.update();
  if (millis() >= next_sensor_read) {
    Serial.print(sensor.getAngle());
    Serial.print("\t");
    Serial.println(sensor.getVelocity());
    next_sensor_read += SENSOR_READ_PERIOD;
  }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(20);
}
