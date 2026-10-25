#pragma once
#include <string>
struct RuntimeOptions {
  std::string backend = "serial";
  std::string config;
  std::string model;
  std::string motor_config;
  std::string trace;
  std::string scenario = "idle";
  double duration = 0;
  int step_us = 500;
  bool joystick = false;
  bool no_joystick = false;
  bool realtime = false;
  bool suspended = false;
  bool pty_server = false;
};
int RunSimulation(const RuntimeOptions& options);
std::string ResolveAsset(const std::string& user_path, const std::string& runfile);
