#include "Arduino.h"
#include "BLDCMotor.h"
#include "SimpleFOC.h"
#include "base/binary_commander.h"
#include "base/communication.h"
#include "base/custom_magnetic_sensor_i2c.h"
#include "drivers/BLDCDriver3PWM.h"

#define MOTOR_AVAILABLE 0x01
#define SENSOR_AVAILABLE 0x02

uint8_t availability[2] = {MOTOR_AVAILABLE & SENSOR_AVAILABLE,
                           MOTOR_AVAILABLE& SENSOR_AVAILABLE};

CustomMagneticSensorI2C sensors[2] = {
    CustomMagneticSensorI2C(AS5600_I2C, A0, A1),
    CustomMagneticSensorI2C(AS5600_I2C, A1, A0)};

BLDCDriver3PWM drivers[2] = {BLDCDriver3PWM(5, 3, 6),
                             BLDCDriver3PWM(9, 11, 10)};
BLDCMotor motors[2] = {BLDCMotor(7), BLDCMotor(7)};

BinaryCommander commander;
ControllerState controller_state = ControllerState::PRE_INIT;
FOCMotor* GetMotor(uint8_t index) { return &motors[index]; }

CustomMagneticSensorI2C* GetSensor(uint8_t index) { return &sensors[index]; }

void Initialize() {}

void InitController() {
  for (int i = 0; i < 2; ++i) {
    if (availability[i] & SENSOR_AVAILABLE) {
      // initialise magnetic sensor hardware
      sensors[i].Activate();
      sensors[i].init();

      // Serial.print(F("Sensor "));
      // Serial.print(i);
      // Serial.println(F(" ready"));
    }

    if (availability[i] & MOTOR_AVAILABLE) {
      drivers[i].voltage_power_supply = 12;
      drivers[i].voltage_limit = 4;
      drivers[i].init();
      drivers[i].enable();

      // Serial.print(F("Driver "));
      // Serial.print(i);
      // Serial.println(F(" ready"));

      motors[i].linkDriver(&drivers[i]);
      if (availability[i] & MOTOR_AVAILABLE) {
        motors[i].linkSensor(&sensors[i]);
      }
      motors[i].controller = MotionControlType::angle;
      motors[i].velocity_limit = 100;
      motors[i].init();
      motors[i].initFOC(motors[i].zero_electric_angle,
                        static_cast<Direction>(motors[i].sensor_direction));

      motors[i].disable();

      // Serial.print(F("Motor "));
      // Serial.print(i);
      // Serial.println(F(" ready"));
    }
  }
}

ControllerState PreInitTick() { return controller_state; }

ControllerState InitTick() {
  InitController();
  return ControllerState::STOPPED;
}

ControllerState StoppedTick() {
  for (int i = 0; i < 2; ++i) {
    if (availability[i] & SENSOR_AVAILABLE) {
      sensors[i].update();
    }
    
    if (availability[i] & MOTOR_AVAILABLE) {
      motors[i].disable();
    }
  }
  return controller_state;
}

ControllerState RunningTick() {
  // TODO: alternate move calls
  for (int i = 0; i < 2; ++i) {
    if (availability[i] & SENSOR_AVAILABLE) {
      sensors[i].update();
    }

    if (availability[i] & MOTOR_AVAILABLE) {
      motors[i].move();
      motors[i].loopFOC();
    }
    return controller_state;
  }
}

ControllerState ErrorTick() { return controller_state; }

void ProcessCommunication() { commander.run(); }

void Tick() {
  ProcessCommunication();

  switch (controller_state) {
    case ControllerState::PRE_INIT:
      controller_state = PreInitTick();
      break;
    case ControllerState::INIT:
      controller_state = InitTick();
      break;
    case ControllerState::STOPPED:
      controller_state = StoppedTick();
      break;
    case ControllerState::RUNNING:
      controller_state = RunningTick();
      break;
    case ControllerState::ERROR:
      controller_state = ErrorTick();
      break;
  }
}

void Critical() {}
