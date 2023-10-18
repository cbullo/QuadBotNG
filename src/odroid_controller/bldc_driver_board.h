#pragma once

#include <assert.h>

#include <atomic>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "src/libs/communication/binary_commands.h"
#include "src/libs/communication/binary_stream.h"
#include "yaml-cpp/yaml.h"

#define MAX_COMMAND_LENGTH 16
#define MAX_CALLBACK_SIZE 32

struct Callback {
  virtual void operator()() = 0;
  virtual void ReadArgs(const uint8_t *buffer, int size) = 0;
};

struct Command {
  Command(){};
  Command(const Command &other) { *this = other; }
  Command &operator=(const Command &other) {
    memcpy(command_bytes, &other.command_bytes, other.command_length);
    command_length = other.command_length;
    memcpy(callback_mem, other.callback_mem, MAX_CALLBACK_SIZE);
    callback = reinterpret_cast<Callback *>(callback_mem);
    return *this;
  }

  uint8_t command_bytes[MAX_COMMAND_LENGTH];
  int command_length = 0;
  uint8_t callback_mem[MAX_CALLBACK_SIZE];
  Callback *callback = nullptr;

  template <class CALLBACK, class FUNCTION>
  void AllocateCallback(FUNCTION f) {
    callback = new (callback_mem) CALLBACK(f);
  }
};

template <class ARG0>
struct SingleArgCallback : public Callback {
  SingleArgCallback(std::function<void(ARG0)> callback_arg)
      : callback(callback_arg){};
  virtual void operator()() override { callback(arg0); }
  virtual void ReadArgs(const uint8_t *buffer, int size) override {
    arg0 = *static_cast<ARG0 *>(buffer);
  }

 private:
  ARG0 arg0 = {};
  std::function<void(ARG0)> callback;
};

template <class ARG0, class ARG1>
struct DoubleArgCallback : public Callback {
  DoubleArgCallback(std::function<void(ARG0, ARG1)> callback_arg)
      : callback(callback_arg){};
  virtual void operator()() override { callback_(arg0, arg1); }
  virtual void ReadArgs(const uint8_t *buffer, int size) override {
    int offset = 0;
    arg0 = *(static_cast<ARG0 *>(buffer) + offset);
    offset += sizeof(ARG0);
    arg1 = *(static_cast<ARG1 *>(buffer) + 1);
  }

 private:
  ARG0 arg0 = {};
  ARG1 arg1 = {};
  std::function<void(ARG0, ARG1)> callback;
};

class BLDCDriverBoard {
 public:
  BLDCDriverBoard();
  void UpdateConfig(const YAML::Node &config);

  void SetMotor(int index, class Motor *m) { motors_[index] = m; }

  bool Connect();
  void Disconnect();
  bool IsConnected() const;
  Motor *GetMotor(int m) { return motors_[m]; }
  const Motor *GetMotor(int m) const { return motors_[m]; }

  std::string GetName() const { return name_; }
  void SetName(const std::string &name) { name_ = name; }

  std::string GetUSBAddress() const { return usb_address_; }

  void ResetBoard();
  void HardwareReset();

  void Tick();

  void SendGetCommand(uint8_t motor_index, COMMAND_TYPE command);
  void SendGetCommand(uint8_t motor_index, COMMAND_TYPE command,
                      COMMAND_TYPE subcommand);

  template <class ARGUMENT>
  void SendSetCommand(uint8_t motor_index, COMMAND_TYPE command,
                      ARGUMENT argument);
  template <class ARGUMENT0, class ARGUMENT1>
  void SendSetCommand(uint8_t motor_index, COMMAND_TYPE command,
                      ARGUMENT0 argument0, ARGUMENT1 argument1);
  template <class ARGUMENT>
  void SendSetCommandAndSub(uint8_t motor_index, COMMAND_TYPE command,
                            COMMAND_TYPE subcommand, ARGUMENT argument);
  template <class ARGUMENT0, class ARGUMENT1>
  void SendSetCommandAndSub(uint8_t motor_index, COMMAND_TYPE command,
                            COMMAND_TYPE subcommand, ARGUMENT0 argument0,
                            ARGUMENT1 argument1);

  void RegisterDataStreamCallback();
  void RegisterStringMsgCallback();

  void SetErrorState(bool enabled) { error_state_ = enabled; }

  bool IsTransmissionBound() const { return is_transmission_bound_; };
  bool AreCommandsScheduled();

  int serial_ = -1;
  void SendSync();

 private:
  enum class ExpectedMessagePart { kHeaderPart, kStaticPart, kDynamicPart };

  void SendCommand(COMMAND_TYPE command);

  void PushCommand(const Command &cmd);

  void StartCommunicationThread();
  void DiscardReceive();
  void ProcessReceive();
  void ProcessSend();
  void ProcessLoop();
  void SendCommand(const uint8_t *command, int length);

  static const int kDefaultCommunicationTimeout = 10000;
  void BumpTimeout(int ms = kDefaultCommunicationTimeout);
  void CheckTimeout();
  void SetState(SyncState new_state);

  uint8_t ProcessHeader(uint8_t header_byte);
  uint8_t CalculateReplySize(uint8_t header_byte);
  uint8_t CalculateDataStreamSize(uint8_t header_byte);
  uint8_t ProcessStaticData(const uint8_t *bytes);
  void ProcessMessage(const uint8_t *message, int message_size);
  void HandleCommunicationError(const std::string &message);

  void RelayMessages();
  void SendSetupVariables();

  std::unique_ptr<std::thread> communication_thread_;

  std::mutex commands_mutex_;
  std::queue<Command> pending_commands_;
  std::queue<Command> sent_get_commands_;

  std::mutex reply_mutex_;
  std::deque<Command> replied_commands_queue_;

  std::chrono::time_point<std::chrono::system_clock> communication_deadline_;
  ExpectedMessagePart msg_part_ = ExpectedMessagePart::kHeaderPart;
  int expected_msg_bytes_ = MESSAGE_HEADER_SIZE;

  static const int kReadBuffer = 256;
  uint8_t serial_read_buffer_[kReadBuffer];

  uint8_t read_offset_ = 0;

  YAML::Node config_;
  std::string usb_address_;

  class Motor *motors_[2] = {nullptr, nullptr};
  std::string name_;

  SyncState sync_state_ = SyncState::kNotSynced;

  bool error_state_ = false;
  std::string error_message_;

  bool is_transmission_bound_ = false;
};

template <class ARGUMENT>
void BLDCDriverBoard::SendSetCommand(uint8_t motor_index, COMMAND_TYPE command,
                                     ARGUMENT argument) {
  Command cmd;
  cmd.command_bytes[0] =
      (command & MAIN_COMMAND_MASK) | (motor_index == 1 ? MOTOR_INDEX_BIT : 0);
  cmd.command_length = 0;
  cmd.command_length += 1;
  const int arg_size = sizeof(argument);
  memcpy(cmd.command_bytes + cmd.command_length, &argument, arg_size);
  cmd.command_length += arg_size;

  assert(cmd.command_length <= MAX_COMMAND_LENGTH);
  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    PushCommand(cmd);
  }
}

template <class ARGUMENT0, class ARGUMENT1>
void BLDCDriverBoard::SendSetCommand(uint8_t motor_index, COMMAND_TYPE command,
                                     ARGUMENT0 argument0, ARGUMENT1 argument1) {
  Command cmd;
  cmd.command_bytes[0] =
      (command & MAIN_COMMAND_MASK) | (motor_index == 1 ? MOTOR_INDEX_BIT : 0);
  cmd.command_length = 0;
  cmd.command_length += 1;
  const int arg_size0 = sizeof(argument0);
  memcpy(cmd.command_bytes + cmd.command_length, &argument0, arg_size0);
  cmd.command_length += arg_size0;
  const int arg_size1 = sizeof(argument1);
  memcpy(cmd.command_bytes + cmd.command_length, &argument1, arg_size1);
  cmd.command_length += arg_size1;

  assert(cmd.command_length <= MAX_COMMAND_LENGTH);
  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    PushCommand(cmd);
  }
}

template <class ARGUMENT>
void BLDCDriverBoard::SendSetCommandAndSub(uint8_t motor_index,
                                           COMMAND_TYPE command,
                                           COMMAND_TYPE subcommand,
                                           ARGUMENT argument) {
  Command cmd;
  cmd.command_bytes[0] =
      (command & MAIN_COMMAND_MASK) | (motor_index == 1 ? MOTOR_INDEX_BIT : 0);
  cmd.command_length = 0;
  cmd.command_length += 1;
  cmd.command_bytes[1] = subcommand;
  cmd.command_length += 1;
  const int arg_size = sizeof(argument);
  memcpy(cmd.command_bytes + cmd.command_length, &argument, arg_size);
  cmd.command_length += arg_size;

  assert(cmd.command_length <= MAX_COMMAND_LENGTH);
  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    PushCommand(cmd);
  }
}

template <class ARGUMENT0, class ARGUMENT1>
void BLDCDriverBoard::SendSetCommandAndSub(uint8_t motor_index,
                                           COMMAND_TYPE command,
                                           COMMAND_TYPE subcommand,
                                           ARGUMENT0 argument0,
                                           ARGUMENT1 argument1) {
  // std::cout << "SSC1" << std::endl;
  Command cmd;
  cmd.command_bytes[0] =
      command & MAIN_COMMAND_MASK | (motor_index == 1 ? MOTOR_INDEX_BIT : 0);
  // std::cout << "SSC2" << std::endl;
  cmd.command_length = 0;
  cmd.command_length += 1;
  cmd.command_bytes[1] = subcommand;
  cmd.command_length += 1;
  // std::cout << "SSC3" << std::endl;
  const int arg_size0 = sizeof(argument0);
  // std::cout << arg_size0 << std::endl;
  // std::cout << cmd.command_length << std::endl;
  memcpy(reinterpret_cast<void *>(cmd.command_bytes + cmd.command_length),
         &argument0, arg_size0);
  // std::cout << "SSC4" << std::endl;
  cmd.command_length += arg_size0;
  const int arg_size1 = sizeof(argument1);
  // std::cout << "SSC5" << std::endl;
  memcpy(reinterpret_cast<void *>(cmd.command_bytes + cmd.command_length),
         &argument1, arg_size1);
  cmd.command_length += arg_size1;
  // std::cout << "SSC6" << std::endl;
  //  std::cout << cmd.command_length << std::endl;

  assert(cmd.command_length <= MAX_COMMAND_LENGTH);
  // std::cout << "SSC7" << std::endl;
  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    PushCommand(cmd);
  }
  // std::cout << "SSC8" << std::endl;
}