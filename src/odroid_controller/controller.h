#pragma once

#include <any>
#include <unordered_map>
#include <vector>

#include "event_factory.h"
#include "leg.h"
// #include "leg_control.h"
#include "leg_controls.h"

class Controller;

class Behavior : public EventNode {
 public:
  Behavior() {}
  virtual ~Behavior(){};
  // virtual void Start(Legs& legs) = 0;
  // virtual void Update(Legs& legs, float dt) = 0;
  void Activate() override;
  void Deactivate() override;

  Behavior* NextBehavior() const { return next_behavior_; }
  Behavior* PreviousBehavior() const { return previous_behavior_; }

  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;

  void AttachActivationCallback(
      const std::function<void(const Behavior&)>& cb) {
    activation_callbacks_.push_back(cb);
  }
  void AttachDectivationCallback(
      const std::function<void(const Behavior&)>& cb) {
    deactivation_callbacks_.push_back(cb);
  };

  void SetController(Controller* controller) { controller_ = controller; };

  void SetNextBehavior(Behavior* next) { next_behavior_ = next; };
  void SetPreviousBehavior(Behavior* previous) {
    previous_behavior_ = previous;
  };

 protected:
  inline Controller* GetController() const { return controller_; }

 private:
  Controller* controller_ = nullptr;
  Behavior* next_behavior_ = nullptr;
  Behavior* previous_behavior_ = nullptr;
  // std::unordered_map<InputSource*, std::any> input_configs_;
  std::vector<std::function<void(const Behavior&)>> activation_callbacks_;
  std::vector<std::function<void(const Behavior&)>> deactivation_callbacks_;
};

class StoppedBehavior : public Behavior {
 public:
  StoppedBehavior(){};
  virtual ~StoppedBehavior(){};
  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;

  void SetPreviousBehavior(Behavior* b) { previous_behavior_ = b; }

 private:
  Behavior* previous_behavior_;
};

class EStoppedBehavior : public Behavior {
 public:
  EStoppedBehavior(){};
  virtual ~EStoppedBehavior(){};
  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;
};


class LegTestingBehavior : public Behavior {
 public:
  LegTestingBehavior(){};
  // void Start(Legs& legs) override;
  // void Update(Legs& legs, float dt) override;

  void Activate() override;
  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;

 private:
  void EnableControlForCurrentLeg(Legs& legs);
  void DisableControlForCurrentLeg(Legs& legs);

  void SendSetupData(Leg& leg);

  ThetaGammaZLegControl leg_control_;

  int selected_leg_ = 0;
  // ThetaGammaScheme joystick_scheme_;
};


class MotorTestingBehavior : public Behavior {
 public:
  MotorTestingBehavior(){};
  // void Start(Legs& legs) override;
  // void Update(Legs& legs, float dt) override;

  void Activate() override;

  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;

 private:
  void EnableControlForCurrentLeg(Legs& legs);
  void DisableControlForCurrentLeg(Legs& legs);

  CalibrationLegControl leg_control_;
  int selected_motor_ = 0;
  uint16_t sensor_values_[64] = {0};
};

class WalkBehavior : public Behavior {
  public:
  void Activate() override;
  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;

  private:
  InitializationLegControl init_control_[4];
};

// class DriveXYZBehavior : public Behavior {
//  public:
//   DriveXYZBehavior(Controller* c) : Behavior(c){};
//   void Start() override{};
//   void Update() override{};
// };

// enum class ControllerMode {
//   kFindLimits = 0,
//   kThetaGammaDrive = 1,
//   kXYZDrive = 2,
//   kWalk = 3
// };

class Controller {
 public:
  Controller(const Legs& legs, const std::vector<ControllerType*>& controllers,
             std::shared_ptr<StoppedBehavior> stopped_behavior,
             std::shared_ptr<EStoppedBehavior> estopped_behavior);

  // void ProcessInput(const ControlEvent& event);

  void Update(float dt);
  inline const Legs& GetLegs() const { return legs_; }
  inline Legs& GetLegs() { return legs_; }

  inline const std::vector<ControllerType*>& GetControllers() const {
    return controllers_;
  }
  inline std::vector<ControllerType*>& GetControllers() { return controllers_; }

  // void SetMode(ControllerMode new_mode);

  // bool ProgressCalibration();
  // void SetThetaGamma(double theta, double gamma, double z);

  // void SetBehavior(Behavior* b);

  // void AttachBehavior(std::unique_ptr<Behavior> behavior);
  // void AttachInputSource(InputSource* source, std::any input_config);

  void AttachBehavior(std::shared_ptr<Behavior> node) {
    node->SetController(this);
    nodes_.push_back(node);
  }

  void AttachEventNode(std::shared_ptr<EventNode> node) {
    nodes_.push_back(node);
  }

  void SubscribeToEvent(EventNode* node, EventId event) {
    subscriptions_.insert({event, node});
  }

  void SubscribeToTick(EventNode* node) { tickables_.push_back(node); }
  float GetDuration() const { return duration_; }

 private:
  void DistributeEvents(const std::deque<ControlEvent>& events);

  Legs legs_;
  std::vector<ControllerType*> controllers_;
  // Behavior* current_behavior_ = {};
  //  std::vector<std::shared_ptr<EventFactory>> control_sources_ = {};

  std::shared_ptr<StoppedBehavior> stopped_behavior_;
  std::shared_ptr<EStoppedBehavior> estopped_behavior_;
  std::vector<std::shared_ptr<EventNode>> nodes_;
  std::vector<EventNode*> tickables_;
  std::unordered_multimap<EventId, EventNode*> subscriptions_;

  float duration_ = 0.f;
  float dt_acc_ = 0.f;

  // DriveThetaGammaBehavior theta_gamma_behavior_;
  // DriveXYZBehavior xyz_behavior_;
};