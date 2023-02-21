#include <avr/wdt.h>

#include "Arduino.h"
// #include "BLDCMotor.h"
// #include "SimpleFOC.h"
#include "base/binary_commander.h"
// #include "base/simple_foc/BLDCDriver3PWM.h"
#include "base/simple_foc/bldc_motor.h"
#include "base/simple_foc/custom_magnetic_sensor_i2c.h"
#include "src/libs/communication/binary_stream.h"

#define MOTOR_AVAILABLE 0x01
#define SENSOR_AVAILABLE 0x02

uint8_t availability[2] = {
    0,
    0
    //0//MOTOR_AVAILABLE | SENSOR_AVAILABLE
};

/** Typical configuration for the 12bit AMS AS5600 magnetic sensor over I2C
 * interface */
MagneticSensorI2CConfig_s AS5600_I2C = {.chip_address = 0x36,
                                        .bit_resolution = 12,
                                        .angle_register = 0x0C,
                                        .data_start_bit = 11};

CustomMagneticSensorI2C sensors[2] = {
    CustomMagneticSensorI2C(AS5600_I2C, A1, A0),
    CustomMagneticSensorI2C(AS5600_I2C, A0, A1)};

// BLDCDriver3PWM drivers[2] = {BLDCDriver3PWM(5, 3, 6),
//                              BLDCDriver3PWM(9, 11, 10)};
BLDCMotor motors[2] = {BLDCMotor(&sensors[0], 6, 5, 3, 7, 1),
                       BLDCMotor(&sensors[1], 9, 11, 10, 7, 1)};

BinaryCommander commander;
ControllerState controller_state = ControllerState::PRE_INIT;
// FOCMotor* GetMotor(uint8_t index) { return &motors[index]; }

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
  }

  for (int i = 0; i < 2; ++i) {
    if (availability[i] & MOTOR_AVAILABLE) {
      motors[i].voltage_power_supply = 12 * 512;
      motors[i].voltage_limit = 12 * 512;
      // motors[i].init();
      // motors[i].enable();
    }
  }

  for (int i = 0; i < 2; ++i) {
    // Serial.print(F("Driver "));
    // Serial.print(i);
    // Serial.println(F(" ready"));

    if (availability[i] & MOTOR_AVAILABLE) {
      // motors[i].linkDriver(&drivers[i]);

      if (availability[i] & (SENSOR_AVAILABLE)) {
        motors[i].linkSensor(&sensors[i]);
      }
      //}
      // motors[i].controller = MotionControlType::angle;
      // motors[i].velocity_limit = 100;

      // motors[i].zero_electric_angle = 328;
      motors[i].sensor_direction = 1;
      motors[i].controller = MotionControlType::voltage;
      // motors[i].sensor_offset = 0;
      // motors[i].foc_modulation = FOCModulationType::SpaceVectorPWM;
      // motors[i].voltage_sensor_align = 12.f;
      motors[i].voltage_limit = 12 * 512 - 1;
      // motors[i].modulation_centered = 0;
      motors[i].init();
      motors[i].enable();

      // wdt_enable(WDTO_8S);
      // for (int j = 0; j < 10; ++j) {
      //   sensors[i].update();
      //   motors[i].sensor_direction = Direction::CW;
      //   motors[i].zero_electric_angle = 0;
      //   motors[i].zero_electric_angle = NOT_SET;
      //   motors[i].initFOC();
      //   char str[10];
      //   volatile int v = static_cast<int>(motors[i].zero_electric_angle *
      //   1000); volatile int v1 = static_cast<int>(motors[i].sensor_direction
      //   * motors[i].pole_pairs); sprintf(str, "%d", v);
      //   commander.SendString(STRING_MESSAGE_TYPE_INFO, str);
      //   sprintf(str, "%d", v1);
      //   commander.SendString(STRING_MESSAGE_TYPE_INFO, str);
      //   _delay(50);
      //   wdt_reset();
      // }

      // motors[i].initFOC();

      // motors[i].init();
      // motors[i].disable();

      // Serial.print(F("Motor "));
      // Serial.print(i);
      // Serial.println(F(" ready"));
    }
  }
  commander.SendString(STRING_MESSAGE_TYPE_INFO, "INIC");
}

ControllerState PreInitTick() { return controller_state; }

ControllerState InitTick() {
  InitController();

  return ControllerState::STOPPED;
}

ControllerState StoppedTick() {
  // commander.SendString(STRING_MESSAGE_TYPE_INFO, "STTT");
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
    if (availability[i] & MOTOR_AVAILABLE) {
      // unsigned long dt = motors[i].timestamp_prev;

      motors[i].loopFOC();
      // motors[i].move();

      // char str[10];
      // volatile unsigned long v = static_cast<unsigned long>(dt);
      // sprintf(str, "%ld", v);
      // commander.SendString(STRING_MESSAGE_TYPE_INFO, str);
    }
  }
  return controller_state;
}

ControllerState ErrorTick() { return controller_state; }

void ProcessCommunication() {
  // Serial.println("A");
  commander.run();
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
