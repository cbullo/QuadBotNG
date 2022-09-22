#pragma once

#include "Arduino.h"
#include "BLDCMotor.h"
#include "base/communication.h"
#include "common/base_classes/FOCMotor.h"
#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "custom_magnetic_sensor_i2c.h"
#include "src/libs/communication/binary_commands.h"

// callback function pointer definiton
typedef void (*BinaryCommandCallback)(
    uint8_t*);  //!< command callback function pointer

class BinaryCommander {
 public:
  BinaryCommander();

  void run();

  void motor(FOCMotor* motor, uint8_t* user_cmd);

  void lpf(LowPassFilter* lpf, uint8_t* user_cmd);
  void pid(PIDController* pid, uint8_t* user_cmd);

 private:
  // Subscribed command callback variables
  // BinaryCommandCallback call_list[12];  //!< array of command callback
  // pointers char call_ids[12];                    //!< added callback commands

  // helper variable for serial communication reading
  uint8_t received_cmd[PRIMARY_COMMANDS_COUNT] = {
      0};           //!< so far received user message - waiting for newline
  int rec_cnt = 0;  //!< number of characters received

  int bytes_to_read = 0;

  void SendString(uint8_t level, const char* message);
  void SendString(uint8_t level, const __FlashStringHelper* message);
  void SendReply(uint8_t cmd, float var, uint8_t sequence);
  void SendDataStream();

  void SendSync();
  void SendPing();

  void CheckAndSendPing();
  void BumpTimeout();

  void send(const float number);
  void send(const uint16_t number);
  void send(const uint8_t number);

  // void run(uint8_t* user_input);
  void process(uint8_t* user_command);

  uint16_t micros_timeout_ = 0;
  bool sync_sent_ = false;
};

enum class ControllerState {
  PRE_INIT = 0,
  INIT = 1,
  STOPPED = 2,
  RUNNING = 3,
  ERROR = 4
};

void ProcessStateCommand(ControllerState new_state);
