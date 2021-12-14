#include "controller.h"

#include <iostream>

Controller::Controller(std::vector<Leg*> legs)
    : legs_(legs), theta_gamma_behavior_(this), xyz_behavior_(this) {}

void Controller::Update() {
  if (current_behavior_) {
    current_behavior_->Update();
  }

  for (auto leg : legs_) {
    leg->UpdateControl();
  }
}

void Controller::SetBehavior(Behavior* b) {
  current_behavior_ = b;
  current_behavior_->Start();
};

void Controller::SetMode(ControllerMode new_mode) {
  switch (new_mode) {
    case ControllerMode::kThetaGammaDrive:
      SetBehavior(&theta_gamma_behavior_);
      break;
    case ControllerMode::kXYZDrive:
      SetBehavior(&xyz_behavior_);
      break;
    case ControllerMode::kWalk:
      // TODO: Waiting for all legs :)
      break;
  }
}

void Controller::SetThetaGamma(double theta, double gamma, double z) {
}