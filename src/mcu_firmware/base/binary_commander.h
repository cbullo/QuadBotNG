#pragma once

#include "Arduino.h"
// #include "common/lowpass_filter.h"
// #include "common/pid.h"
#include "src/libs/communication/binary_commands.h"
#include "src/mcu_firmware/base/simple_foc/bldc_motor.h"


#define MESSAGE_TYPE_GET_REPLY 0x40

// callback function pointer definiton
typedef void (*BinaryCommandCallback)(
    uint8_t*);  //!< command callback function pointer

class BinaryCommander {
 public:
  BinaryCommander();

  void run();

  void motor(BLDCMotor* motor, uint8_t* user_cmd);

  // void lpf(LowPassFilter* lpf, uint8_t* user_cmd);
  // void pid(PIDController* pid, uint8_t* user_cmd);

  void SendString(uint8_t level, const char* message);
  void SendString(uint8_t level, const __FlashStringHelper* message);

 private:
  // Subscribed command callback variables
  // BinaryCommandCallback call_list[12];  //!< array of command callback
  // pointers char call_ids[12];                    //!< added callback commands

  // helper variable for serial communication reading
  uint8_t received_cmd[PRIMARY_COMMANDS_COUNT] = {
      0};               //!< so far received user message - waiting for newline
  uint8_t rec_cnt = 0;  //!< number of characters received

  uint8_t bytes_to_read = 0;

  //  void SendReply(uint8_t cmd, float var, uint8_t sequence);
  template <class T>
  void SendReply(uint8_t cmd, T var, uint8_t sequence) {
    Serial.write(cmd | MESSAGE_TYPE_GET_REPLY);
    Serial.write(reinterpret_cast<uint8_t*>(&var), sizeof(T));
    Serial.write(&sequence, 1);
    BumpTimeout();
  }

  void SendDataStream();

  void SendSync();
  void SendPing();

  void CheckAndSendPing();
  void BumpTimeout();

  //  void send(const float number);
  void send(const uint16_t number);
  void send(const uint8_t number);

  // void run(uint8_t* user_input);
  void process(uint8_t* user_command);

  unsigned long micros_timeout_ = 0;
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
