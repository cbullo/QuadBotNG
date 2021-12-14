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
#include "../../custom_magnetic_sensor_i2c.h"

// magnetic sensor instance - SPI
CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);
// magnetic sensor instance - MagneticSensorI2C
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 3, 6);
BLDCMotor motor = BLDCMotor(7);
// Stepper motor & driver instance
// StepperMotor motor = StepperMotor(50);
// StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  // use monitoring with serial
  Serial.begin(115200);

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
  // maximal voltage to be set to the motor
  motor.voltage_limit = 10;

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
  // motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('M', doMotor, "motor settings");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);

  motor.disable();
}

unsigned long prev_us;
void loop() {
  unsigned long now_us = _micros();
  // Serial.println(now_us - prev_us);
  prev_us = now_us;

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}

int main() {
  setup();
  while (true) {
    loop();
  }
  return 0;
}