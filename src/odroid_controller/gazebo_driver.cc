#include "gazebo_driver.h"

GazeboDriver::GazeboDriver(){};
void UpdateConfig(const YAML::Node &config) {
  gazebo_host_ = config["gazebo_host"].as<std::string>();
  gazebo_port_ = config["gazebo_port"].as<unsigned int>();
}

void SetMotor(int index, class Motor *m) { motors_[index] = m; }

bool Connect() {
  ConnectionManager::Instance()->Init(gazebo_host_, gazebo_port_);
  
}
void Disconnect();
bool IsConnected() const;
Motor *GetMotor(int m) { return motors_[m]; }
const Motor *GetMotor(int m) const { return motors_[m]; }

std::string GetName() const { return name_; }
void SetName(const std::string &name) { name_ = name; }

void SetErrorState(bool){};

void Tick();

