#include "Arduino.h"
#include "SimpleFOC.h"
//#include "base/custom_magnetic_sensor_i2c.h"

void Critical() {}

#define POLE_PAIR_NUMBER 7

unsigned long next_sensor_read;

//CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

Commander commander = Commander(Serial, '\n', false);

bool enabled = false;


// BLDC motor & driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 11, 10);
BLDCMotor motor = BLDCMotor(POLE_PAIR_NUMBER);

void on_start(char *cmd) {
  enabled = true;
  Serial.println(F("MOTOR STARTED"));
  motor.enable();
}

void on_calib_data1(char *cmd) {
  switch (cmd[0]) {
    case 'C': {
      uint8_t index = 0;
      if (cmd[1] == '1') {
        index += 10;
      }
      if ('0' <= cmd[2] && cmd[2] <= '9') {
        index += cmd[2] - '0';
      }
      if (index <= 15) {
        float value = 0;
        commander.scalar(&value, &cmd[3]);
        //sensor.linearization_.coeffs_[index] = value;
      }
      break;
    }
    case 'O': {
      float value = 0;
      commander.scalar(&value, &cmd[1]);
      //sensor.linearization_.offset = value;
      break;
    }
  }
}

void Initialize() {
  // initialise magnetic sensor hardware
  // sensor.Activate();
  // sensor.init();
  // link the motor to the sensor
  //motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;

  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  motor.voltage_limit = 7;
  motor.velocity_limit = 100;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity_openloop;

  motor.foc_modulation = FOCModulationType::SinePWM;

  // initialize motor
  motor.init();

  Serial.println(F("Motor ready."));

  //next_sensor_read = millis();

  //motor.disable();

  commander.add('E', on_calib_data1, "encoder 1");
  commander.add('S', on_start, "start");

  enabled = true;
  motor.enable();
}

#define SENSOR_READ_PERIOD 100;  // ms

bool activated = false;

void Tick() {
  Serial.println(F("mmove."));
  //commander.run();

  //sensor.update();
  // if (millis() >= next_sensor_read) {
  //   //Serial.print(sensor.getAngle());
  //   //Serial.print("\t");
  //   //Serial.println(sensor.getVelocity());
  //   next_sensor_read += SENSOR_READ_PERIOD;
  // }

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code

  if (enabled) {
    motor.move(100);
    Serial.println(F("move."));
  }
}
