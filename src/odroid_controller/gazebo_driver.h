#pragma once

#include "src/libs/communication/binary_commands.h"
#include "yaml-cpp/yaml.h"

#include "gazebo-11/gazebo/transport/Node.hh"

class GazeboDriver {
 public:
  GazeboDriver();
  void UpdateConfig(const YAML::Node &config);

  void SetMotor(int index, class Motor *m) { motors_[index] = m; }

  bool Connect() {
    gazebo::Node
  }
  void Disconnect();
  bool IsConnected() const;
  Motor *GetMotor(int m) { return motors_[m]; }
  const Motor *GetMotor(int m) const { return motors_[m]; }

  std::string GetName() const { return name_; }
  void SetName(const std::string &name) { name_ = name; }

  void SetErrorState(bool){};

  void Tick();

  void SendGetCommand(uint8_t motor_index, COMMAND_TYPE command);
  void SendGetCommand(uint8_t motor_index, COMMAND_TYPE command,
                      COMMAND_TYPE subcommand);

  template <class ARGUMENT>
  void SendSetCommand(uint8_t motor_index, COMMAND_TYPE command,
                      ARGUMENT argument) {
    switch (command & MAIN_COMMAND_MASK) {
      case CMD_MOTOR_VOLTAGE:
        gazebo... break;
    }
  }
  template <class ARGUMENT0, class ARGUMENT1>
  void SendSetCommand(uint8_t motor_index, COMMAND_TYPE command,
                      ARGUMENT0 argument0, ARGUMENT1 argument1) {}
  template <class ARGUMENT>
  void SendSetCommandAndSub(uint8_t motor_index, COMMAND_TYPE command,
                            COMMAND_TYPE subcommand, ARGUMENT argument) {}
  template <class ARGUMENT0, class ARGUMENT1>
  void SendSetCommandAndSub(uint8_t motor_index, COMMAND_TYPE command,
                            COMMAND_TYPE subcommand, ARGUMENT0 argument0,
                            ARGUMENT1 argument1) {}

 private:
  std::string name_;
  std::string gazebo_host_;
  unsigned int gazebo_port_;
  class Motor *motors_[2] = {nullptr, nullptr};
};