#pragma once

#include <vector>

#include "leg.h"

class Controller;

class Behavior {
 public:
  Behavior(Controller* c) : controller_(c) {}
  virtual void Start() = 0;
  virtual void Update() = 0;

  inline Controller* GetController() { return controller_; }

 private:
  Controller* controller_;
};

class DriveThetaGammaBehavior : public Behavior {
 public:
  DriveThetaGammaBehavior(Controller* c) : Behavior(c){};
  void Start() override{};
  void Update() override{};
};

class DriveXYZBehavior : public Behavior {
 public:
  DriveXYZBehavior(Controller* c) : Behavior(c){};
  void Start() override{};
  void Update() override{};
};

enum class ControllerMode {
  kFindLimits = 0,
  kThetaGammaDrive = 1,
  kXYZDrive = 2,
  kWalk = 3
};

class Controller {
 public:
  Controller(std::vector<Leg*> legs);

  void Update();

  inline const std::vector<Leg*> GetLegs() const { return legs_; }

  void SetMode(ControllerMode new_mode);

  bool ProgressCalibration();
  void SetThetaGamma(double theta, double gamma, double z);

 private:
  void SetBehavior(Behavior* b);
  std::vector<Leg*> legs_;

  Behavior* current_behavior_ = {};

  DriveThetaGammaBehavior theta_gamma_behavior_;
  DriveXYZBehavior xyz_behavior_;
};