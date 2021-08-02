#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <vector>

#include "controller.h"
#include "controller_board.h"
#include "leg.h"

volatile bool estop_triggered = false;
void EStop() { estop_triggered = true; }

std::vector<ControllerBoard*> controllers;
std::vector<Motor*> motors;
std::vector<Leg*> legs;

std::unique_ptr<Controller> main_controller;

void ReadControllers(
    const YAML::Node& config,
    std::unordered_map<std::string, ControllerBoard*>* controllers) {
  const YAML::Node& controllers_yaml = config["controllers"];
  for (auto& controller_yaml : controllers_yaml) {
    auto name = controller_yaml.first.as<std::string>();
    auto controller = new ControllerBoard();
    controller->SetName(name);
    controller->UpdateConfig(controller_yaml.second);
    (*controllers)[name] = controller;
  }
}

void ReadMotors(
    const YAML::Node& config,
    const std::unordered_map<std::string, ControllerBoard*>& controllers,
    std::unordered_map<std::string, Motor*>* motors) {
  const YAML::Node& motors_yaml = config["motors"];
  for (auto& motor_yaml : motors_yaml) {
    auto name = motor_yaml.first.as<std::string>();
    auto controller_name = motor_yaml.second["controller"].as<std::string>();
    auto controller = controllers.at(controller_name);
    auto index = motor_yaml.second["index"].as<int>();
    auto motor = new Motor(controller, index);
    motor->SetName(name);
    motor->UpdateConfig(motor_yaml.second);
    (*motors)[name] = motor;
    if (controller) {
      controller->SetMotor(index, motor);
    }
  }
}

void ReadLegs(const YAML::Node& config,
              const std::unordered_map<std::string, Motor*>& motors,
              std::unordered_map<std::string, Leg*>* legs) {
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
    (*legs)[name] = leg;
  }
}

int main() {
  bool trigger_estop = true;

  auto device = "/dev/input/js0";
  int joystick_fd = open(device, O_RDONLY | O_NONBLOCK);
  if (joystick_fd == -1) {
    std::cout << "Can't open joystick." << std::endl;
    trigger_estop = true;
  }

  while (true) {
    while (true) {
      int bytes;
      js_event event;
      bytes = read(joystick_fd, &event, sizeof(event));

      if (bytes == -1) {
        if (errno != EAGAIN) {
          trigger_estop = true;
        }
        break;
      }

      if (event.type == JS_EVENT_BUTTON && event.value == 0x1) {
        trigger_estop = true;
        break;
      }
    }
    if (trigger_estop) {
      EStop();
    }
  }

  return 0;
}