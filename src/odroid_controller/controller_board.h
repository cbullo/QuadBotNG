#pragma once

#include <atomic>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "yaml-cpp/yaml.h"

class Motor;

class ControllerBoard {
 public:
  ControllerBoard();
  void UpdateConfig(const YAML::Node& config);

  void SetMotor(int index, Motor* m) { motors_[index] = m; }

  bool Connect();
  void Disconnect();
  bool IsConnected() const;
  Motor* GetMotor(int m) { return motors_[m]; }
  const Motor* GetMotor(int m) const { return motors_[m]; }

  std::string GetName() const { return name_; }
  void SetName(const std::string& name) { name_ = name; }

  std::string GetUSBAddress() const { return usb_address_; }

  void ProcessLoop();

  void HardwareReset() const;

  void SendCommand(const std::string& command) const;

 private:
  void StartProcessThread();

  std::unique_ptr<std::thread> communication_thread;

  std::mutex commands_mutex;

  YAML::Node config_;
  std::string usb_address_;
  int serial_ = -1;

  Motor* motors_[2];
  std::string name_;
};
