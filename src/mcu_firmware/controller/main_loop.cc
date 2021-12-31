#include "Arduino.h"
#include "BLDCMotor.h"
#include "SimpleFOC.h"
#include "base/communication.h"
#include "base/custom_magnetic_sensor_i2c.h"
#include "drivers/BLDCDriver3PWM.h"

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t
// _angle_register_msb)
//  chip_address  I2C chip address
//  bit_resolution  resolution of the sensor
//  angle_register_msb  angle read register msb
//  bits_used_msb  number of used bits in msb register
//
// make sure to read the chip address and the chip angle register msb value from
// the datasheet also in most cases you will need external pull-ups on SDA and
// SCL lines!!!!!
//
// For AS5058B
// MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// Example of AS5600 configuration

CustomMagneticSensorI2C sensors[2] = {
    CustomMagneticSensorI2C(AS5600_I2C, A0, A1),
    CustomMagneticSensorI2C(AS5600_I2C, A1, A0)};
BLDCDriver3PWM drivers[2] = {BLDCDriver3PWM(5, 3, 6),
                             BLDCDriver3PWM(9, 11, 10)};
BLDCMotor motors[2] = {BLDCMotor(7), BLDCMotor(7)};

enum class ControllerState {
  PRE_INIT = 0,
  INIT = 1,
  STOPPED = 2,
  RUNNING = 3,
  ERROR = 4
} controller_state = ControllerState::PRE_INIT;

void Initialize() { InitCommunication(); }

void InitController() {
  for (int i = 0; i < 2; ++i) {
    // initialise magnetic sensor hardware
    sensors[i].Activate();
    sensors[i].init();

    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.println(F(" ready"));

    drivers[i].voltage_power_supply = 28;
    drivers[i].voltage_limit = 4;
    drivers[i].init();
    drivers[i].enable();

    Serial.print(F("Driver "));
    Serial.print(i);
    Serial.println(F(" ready"));

    motors[i].linkDriver(&drivers[i]);
    motors[i].controller = MotionControlType::angle;
    motors[i].velocity_limit = 100;
    motors[i].init();
    motors[i].initFOC(motors[i].zero_electric_angle,
                      static_cast<Direction>(motors[i].sensor_direction));

    motors[i].disable();

    Serial.print(F("Motor "));
    Serial.print(i);
    Serial.println(F(" ready"));
  }
}

ControllerState PreInitTick() { return controller_state; }

ControllerState InitTick() {
  InitController();
  return ControllerState::STOPPED;
}

ControllerState StoppedTick() {
  motors[0].disable();
  motors[1].disable();
  return controller_state;
}

ControllerState RunningTick() {
  // TODO: alternate move calls
  motors[0].move();
  motors[1].move();

  motors[0].loopFOC();
  motors[1].loopFOC();
  return controller_state;
}

ControllerState ErrorTick() { return controller_state; }

void ProcessStateCommand(char *cmd) {
  if (cmd[0] == '\n') {
    Serial.println(static_cast<uint8_t>(controller_state));
    return;
  }

  switch (controller_state) {
    case ControllerState::PRE_INIT:
      if (cmd[1] == 'I') {
        controller_state = ControllerState::INIT;
        return;
      }
      break;
    case ControllerState::INIT:
      break;
    case ControllerState::STOPPED:
      if (cmd[1] == 'R') {
        motors[0].enable();
        motors[1].enable();
        controller_state = ControllerState::RUNNING;
        return;
      }
      break;
    case ControllerState::RUNNING:
      if (cmd[1] == 'S') {
        controller_state = ControllerState::STOPPED;
        return;
      }
      break;
    case ControllerState::ERROR:
      break;
  }

  Serial.print(F("Unhandled state command: "));
  Serial.println(cmd[1]);
}

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