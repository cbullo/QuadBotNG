#pragma once
#include <chrono>
#include <map>
#include <string>
#include "robot_system.h"

// Optional loopback-only JSON-lines endpoint. All methods run on the owning
// controller thread; bounded nonblocking I/O cannot wait for a dashboard.
class Dashboard {
 public:
  Dashboard(int port, bool allow_control, std::string backend);
  ~Dashboard();
  Dashboard(const Dashboard&) = delete;
  Dashboard& operator=(const Dashboard&) = delete;
  void Pump(RobotSystem& robot, double time);
  bool Overridden() const { return overridden_; }
  std::string Snapshot(const RobotSystem& robot, double time) const;
 private:
  void Disconnect(RobotSystem& robot);
  void Command(RobotSystem& robot, const std::string& line);
  int listener_ = -1, client_ = -1;
  bool allow_control_, remote_hold_ = false, overridden_ = false;
  std::string backend_, input_, output_, reason_;
  uint64_t sequence_ = 0;
  double next_snapshot_ = 0;
  std::chrono::steady_clock::time_point lease_;
  std::map<std::string, LegConfiguration> origins_;
};
