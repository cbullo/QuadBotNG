#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "bldc_driver_board.h"
#include "controller.h"
#include "leg.h"

volatile bool estop_triggered = false;
bool stopped = true;

bool select_pressed = false;
bool start_pressed = false;

std::vector<BLDCDriverBoard*> controllers;
std::vector<Motor*> motors;
std::vector<Leg*> legs;
std::unique_ptr<Controller> main_controller;

struct axis_state {
  short x, y;
};

void EStop() {
  if (!estop_triggered) {
    std::cout << "EStop triggered!" << std::endl;
    estop_triggered = true;
  }
}

void ReleaseEStop() {
  estop_triggered = false;
  std::cout << "EStop released!" << std::endl;
}

void Run() {
  for (const auto& controller : controllers) {
    controller->SendSetCommand(0, CMD_STATE, static_cast<uint8_t>(3));
    std::cout << "Running!" << std::endl;
  }

  stopped = false;
}

void Stop() {
  for (const auto& controller : controllers) {
    // controller->SendCommand("S");
    controller->SendSetCommand(0, CMD_STATE, static_cast<uint8_t>(2));
  }
  std::cout << "Stopped!" << std::endl;
  stopped = true;
}

void ReadControllers(
    const YAML::Node& config,
    std::unordered_map<std::string, BLDCDriverBoard*>& controllers) {
  const YAML::Node& controllers_yaml = config["controllers"];
  for (auto& controller_yaml : controllers_yaml) {
    auto name = controller_yaml.first.as<std::string>();
    std::cout << "Reading " << name << std::endl;
    auto controller = new BLDCDriverBoard();
    controller->SetName(name);
    controller->UpdateConfig(controller_yaml.second);
    controllers[name] = controller;
  }
}

void ReadMotors(
    const YAML::Node& config,
    const std::unordered_map<std::string, BLDCDriverBoard*>& controllers,
    std::unordered_map<std::string, Motor*>& motors) {
  const YAML::Node& motors_yaml = config["motors"];
  for (auto& motor_yaml : motors_yaml) {
    auto name = motor_yaml.first.as<std::string>();
    auto controller_name = motor_yaml.second["controller"].as<std::string>();
    auto controller = controllers.at(controller_name);
    auto index = motor_yaml.second["index"].as<int>();
    auto motor = new Motor(controller, index);
    motor->SetName(name);
    motor->UpdateConfig(motor_yaml.second);
    motors[name] = motor;
    if (controller) {
      controller->SetMotor(index, motor);
    }
  }
}

void ReadLegs(const YAML::Node& config,
              const std::unordered_map<std::string, Motor*>& motors,
              std::unordered_map<std::string, Leg*>& legs) {
  const YAML::Node& legs_yaml = config["legs"];
  for (auto& leg_yaml : legs_yaml) {
    auto name = leg_yaml.first.as<std::string>();

    auto motor_F_name = leg_yaml.second["motor_F"].as<std::string>();
    auto motor_B_name = leg_yaml.second["motor_B"].as<std::string>();
    auto motor_Z_name = leg_yaml.second["motor_Z"].as<std::string>();

    auto m_f = motors.at(motor_F_name);
    auto m_b = motors.at(motor_B_name);
    auto m_z = motors.at(motor_Z_name);

    auto leg = new Leg(m_f, m_b, m_z);
    leg->UpdateConfig(leg_yaml.second);
    legs[name] = leg;
  }
}

size_t get_axis_state(struct js_event* event, axis_state axes[3]) {
  size_t axis = event->number;

  if (axis < 6) {
    axes[axis].x = event->value;
  }

  return axis;
}

int main() {
  std::cout << "Running controller" << std::endl;
  axis_state axes[6] = {0};

  YAML::Node config = YAML::LoadFile("config/robot_config.yaml");

  std::unordered_map<std::string, BLDCDriverBoard*> controllers_map;
  std::unordered_map<std::string, Motor*> motors_map;

  ReadControllers(config, controllers_map);
  ReadMotors(config, controllers_map, motors_map);

  std::transform(controllers_map.begin(), controllers_map.end(),
                 back_inserter(controllers),
                 [](const auto& val) { return val.second; });

  for (const auto& controller : controllers) {
    if (!controller->Connect()) {
      std::cout << "Failed to connect to controller " << controller->GetName()
                << std::endl;
    } else {
      std::cout << "Connected to controller " << controller->GetName()
                << std::endl;
    }
  }

  bool trigger_estop = false;

  auto device = "/dev/input/js0";

  int joystick_fd = -1;

  while (true) {
    if (joystick_fd == -1) {
      joystick_fd = open(device, O_RDONLY | O_NONBLOCK);
      if (joystick_fd == -1) {
        // std::cout << "Can't connect to joystick.." << std::endl;
        trigger_estop = true;
      }
    }

    while (joystick_fd != -1) {
      int bytes;
      js_event event;
      bytes = read(joystick_fd, &event, sizeof(event));

      if (bytes == -1) {
        if (errno != EAGAIN) {
          trigger_estop = true;
          close(joystick_fd);
          joystick_fd = -1;
        }
        break;
      }

      if (event.type == JS_EVENT_BUTTON) {
        if (estop_triggered) {
          if (event.number == 0x0) {
            select_pressed = event.value;
          } else if (event.number == 0x3) {
            start_pressed = event.value;
          }
        } else {
          if (event.number == 0x0 && event.value == 1) {
            trigger_estop = true;
            break;
          } else if (event.number == 0x3 && event.value == 1) {
            if (stopped) {
              Run();
            } else {
              Stop();
            }
          }
        }
      } else if (event.type == JS_EVENT_AXIS) {
        size_t axis = get_axis_state(&event, axes);
        if (axis < 6) {
          double x = axes[1].x / 32768.0;
          double y = axes[0].x / 32768.0;
          double z = axes[3].x / 32768.0;
          auto theta = atan2(y, x);
          auto gamma = sqrt(x * x + y * y);
          // if (gamma > 0.1) {
          //   main_controller->SetThetaGamma(theta, gamma, z);
          // }

          for (int i = 0; i < 2; ++i) {
            for (const auto& controller : controllers) {
              int16_t target0 = static_cast<int16_t>(512 * (12 * x));
              int16_t target1 = static_cast<int16_t>(512 * (12 * y));
              // std::cout << "Sending targets: " << target0 << " " << target1
              //           << std::endl;
              controller->SendSetCommand(0, CMD_MOTOR_VOLTAGE, target0);
              controller->SendSetCommand(1, CMD_MOTOR_VOLTAGE, target1);
              // controller->SendSetCommand(1, CMD_MOTOR_TARGET,
              // static_cast<float>(100.f * gamma));
            }
          }
        }
      }
    }

    if (trigger_estop) {
      EStop();
      trigger_estop = false;
    }

    if (estop_triggered) {
      if (select_pressed && start_pressed) {
        ReleaseEStop();
        select_pressed = false;
        start_pressed = false;
      }

      for (const auto& controller : controllers) {
        controller->HardwareReset();
      }
      usleep(100000);
    }
  }

  return 0;
}