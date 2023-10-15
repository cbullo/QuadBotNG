#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#if QUADBOT_SIMULATOR
#include "gazebo_driver.h"
#else
#include "src/odroid_controller/bldc_driver_board.h"
#endif

#include "src/odroid_controller/controller.h"
#include "src/odroid_controller/joystick_input.h"
#include "src/odroid_controller/leg.h"
#include "src/odroid_controller/web_server/server.h"

volatile bool estop_triggered = false;
bool stopped = true;

bool select_pressed = false;
bool start_pressed = false;

#ifdef QUADBOT_SIMULATOR
using ControllerType = GazeboDriver;
#else
using ControllerType = BLDCDriverBoard;
#endif

std::vector<ControllerType*> controllers;
std::vector<Motor*> motors;
std::vector<Leg*> legs;
// std::unique_ptr<Controller> main_controller;

WebServer web_server;

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
    std::unordered_map<std::string, ControllerType*>& controllers) {
  const YAML::Node& controllers_yaml = config["controllers"];
  for (auto& controller_yaml : controllers_yaml) {
    auto name = controller_yaml.first.as<std::string>();
    std::cout << "Reading " << name << std::endl;
    auto controller = new ControllerType();
    controller->SetName(name);
    controller->UpdateConfig(controller_yaml.second);
    controllers[name] = controller;
  }
}

void ReadMotors(
    const YAML::Node& config,
    const std::unordered_map<std::string, ControllerType*>& controllers,
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
              std::map<std::string, Leg*>& legs) {
  const YAML::Node& legs_yaml = config["legs"];
  for (auto& leg_yaml : legs_yaml) {
    auto name = leg_yaml.first.as<std::string>();

    Motor* m_i = nullptr;
    Motor* m_o = nullptr;
    Motor* m_z = nullptr;

    if (YAML::Node motor_node = leg_yaml.second["motor_I"]) {
      auto motor_I_name = motor_node.as<std::string>();
      m_i = motors.at(motor_I_name);
    }

    if (YAML::Node motor_node = leg_yaml.second["motor_O"]) {
      auto motor_O_name = motor_node.as<std::string>();
      m_o = motors.at(motor_O_name);
    }

    if (YAML::Node motor_node = leg_yaml.second["motor_Z"]) {
      auto motor_Z_name = motor_node.as<std::string>();
      m_z = motors.at(motor_Z_name);
    }

    auto leg = new Leg(m_i, m_o, m_z);
    leg->SetName(name);
    leg->UpdateConfig(leg_yaml.second);
    legs[name] = leg;
  }
}

Controller SetupController() {
  std::cout << "SetupController 1" << std::endl;
  auto device = "/dev/input/js0";

  auto stopped_behavior = std::make_shared<StoppedBehavior>();
  auto estopped_behavior = std::make_shared<EStoppedBehavior>();

  auto estopped_scheme = std::make_shared<EStoppedScheme>();
  auto stopped_scheme = std::make_shared<StoppedScheme>();
  auto calibration_scheme = std::make_shared<CalibrationScheme>();
  auto thetagamma_scheme = std::make_shared<ThetaGammaScheme>();
  auto walk_scheme = std::make_shared<WalkScheme>();

  auto joystick_input = std::make_shared<JoystickInput>(device);

  auto leg_testing_behavior = std::make_shared<LegTestingBehavior>();
  auto motor_testing_behavior = std::make_shared<MotorTestingBehavior>();
  auto walk_behavior = std::make_shared<WalkBehavior>();

  // motor_testing_behavior->SetPreviousBehavior(walk_behavior.get());
  // motor_testing_behavior->SetNextBehavior(leg_testing_behavior.get());

  // leg_testing_behavior->SetPreviousBehavior(motor_testing_behavior.get());
  // leg_testing_behavior->SetNextBehavior(walk_behavior.get());

  walk_behavior->SetPreviousBehavior(leg_testing_behavior.get());
  walk_behavior->SetNextBehavior(motor_testing_behavior.get());

  joystick_input->Init();

  stopped_behavior->AttachActivationCallback(
      [joystick_input, stopped_scheme](const Behavior& b) mutable {
        joystick_input->SetScheme(stopped_scheme);
      });

  estopped_behavior->AttachActivationCallback(
      [joystick_input, estopped_scheme](const Behavior& b) mutable {
        joystick_input->SetScheme(estopped_scheme);
      });

  leg_testing_behavior->AttachActivationCallback(
      [joystick_input, thetagamma_scheme](const Behavior& b) mutable {
        std::cout << "Setting theta gamma scheme" << std::endl;
        joystick_input->SetScheme(thetagamma_scheme);
      });

  motor_testing_behavior->AttachActivationCallback(
      [joystick_input, calibration_scheme](const Behavior& b) mutable {
        std::cout << "Setting calibration scheme" << std::endl;
        joystick_input->SetScheme(calibration_scheme);
      });

  walk_behavior->AttachActivationCallback(
      [joystick_input, walk_scheme](const Behavior& b) mutable {
        std::cout << "Setting walk scheme" << std::endl;
        joystick_input->SetScheme(walk_scheme);
      });

  Legs legs_s;
  legs_s.bl = legs.size() > 0 ? legs[0] : nullptr;
  legs_s.br = legs.size() > 1 ? legs[1] : nullptr;
  legs_s.fl = legs.size() > 2 ? legs[2] : nullptr;
  legs_s.fr = legs.size() > 3 ? legs[3] : nullptr;

  Controller main_controller(legs_s, controllers, stopped_behavior,
                             estopped_behavior);

  main_controller.AttachEventNode(joystick_input);
  // main_controller.AttachBehavior(leg_testing_behavior);
  // main_controller.AttachBehavior(motor_testing_behavior);
  main_controller.AttachBehavior(walk_behavior);

  main_controller.SubscribeToTick(joystick_input.get());

  main_controller.SubscribeToEvent(leg_testing_behavior.get(),
                                   EventId::kControlEventConfirm);
  main_controller.SubscribeToEvent(leg_testing_behavior.get(),
                                   EventId::kControlEventNextItem);
  main_controller.SubscribeToEvent(leg_testing_behavior.get(),
                                   EventId::kControlEventPreviousItem);
  main_controller.SubscribeToEvent(leg_testing_behavior.get(),
                                   EventId::kControlEventLegTheta);
  main_controller.SubscribeToEvent(leg_testing_behavior.get(),
                                   EventId::kControlEventLegGamma);
  main_controller.SubscribeToEvent(leg_testing_behavior.get(),
                                   EventId::kControlEventLegTilt);

  main_controller.SubscribeToEvent(motor_testing_behavior.get(),
                                   EventId::kControlEventNextMode);
  main_controller.SubscribeToEvent(motor_testing_behavior.get(),
                                   EventId::kControlEventPreviousMode);
  main_controller.SubscribeToEvent(motor_testing_behavior.get(),
                                   EventId::kControlEventNextItem);
  main_controller.SubscribeToEvent(motor_testing_behavior.get(),
                                   EventId::kControlEventPreviousItem);
  main_controller.SubscribeToEvent(motor_testing_behavior.get(),
                                   EventId::kControlEventConfirm);
  main_controller.SubscribeToEvent(motor_testing_behavior.get(),
                                   EventId::kControlEventLegTilt);

  main_controller.SubscribeToEvent(walk_behavior.get(),
                                   EventId::kControlEventConfirm);

  stopped_behavior->SetPreviousBehavior(walk_behavior.get());
  stopped_behavior->Activate();
  std::cout << "SetupController 2" << std::endl;

  web_server.Init(&main_controller);

  return main_controller;
}

int main() {
  std::cout << "Running controller" << std::endl;

  YAML::Node config = YAML::LoadFile("config/robot_config.yaml");

  std::unordered_map<std::string, ControllerType*> controllers_map;
  std::unordered_map<std::string, Motor*> motors_map;
  std::map<std::string, Leg*> legs_map;

  ReadControllers(config, controllers_map);
  ReadMotors(config, controllers_map, motors_map);
  ReadLegs(config, motors_map, legs_map);

  std::transform(controllers_map.begin(), controllers_map.end(),
                 back_inserter(controllers),
                 [](const auto& val) { return val.second; });

  std::transform(legs_map.begin(), legs_map.end(), back_inserter(legs),
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

  // bool trigger_estop = false;

  // auto device = "/dev/input/js0";

  // int joystick_fd = -1;

  auto main_controller = SetupController();

  std::cout << "Before loop" << std::endl;
  auto prev_time = std::chrono::high_resolution_clock::now();
  while (true) {
    auto curr_time = std::chrono::high_resolution_clock::now();
    float elapsed_time_s =
        std::chrono::duration<float>(curr_time - prev_time).count();
    main_controller.Update(elapsed_time_s);
    prev_time = curr_time;

    // if (trigger_estop) {
    //   EStop();
    //   trigger_estop = false;
    // }

    // if (estop_triggered) {
    //   if (select_pressed && start_pressed) {
    //     ReleaseEStop();
    //     select_pressed = false;
    //     start_pressed = false;
    //   }

    //   for (const auto& controller : controllers) {
    //     controller->HardwareReset();
    //   }
    //   usleep(100000);
    // }
  }

  return 0;
}