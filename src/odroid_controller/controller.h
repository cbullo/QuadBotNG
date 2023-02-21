#pragma once

#include <any>
#include <unordered_map>
#include <vector>

#include "event_factory.h"
#include "leg.h"
#include "leg_control.h"

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
  void SetPreviousBehavior(Behavior* previous) { previous_behavior_ = previous; };

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

class ThetaGammaZLegControl : public LegControl {
 public:
  bool Process(Leg& leg, float dt) override;

  void SetThetaSetpoint(float theta) { state_.theta_setpoint = theta; }
  void SetGammaSetpoint(float gamma) { state_.gamma_setpoint = gamma; }
  void SetZSetpoint(float z) { state_.z_setpoint = z; }

 private:
  struct State {
    float theta_setpoint = 0.0;
    float gamma_setpoint = 0.0;
    float z_setpoint = 0.0;

    float prev_theta_error = 0.0;
    float prev_gamma_error = 0.0;
    float prev_z_error = 0.0;
    float i_theta_sum = 0.0;
    float i_gamma_sum = 0.0;
    float i_z_sum = 0.0;
  };
  State state_;

  float delay_ = -0.01f;
};

class LegTestingBehavior : public Behavior {
 public:
  LegTestingBehavior(){};
  // void Start(Legs& legs) override;
  // void Update(Legs& legs, float dt) override;

  void ProcessInputEvents(const std::deque<ControlEvent>& events) override;

 private:
  void EnableControlForCurrentLeg(Legs& legs);
  void DisableControlForCurrentLeg(Legs& legs);

  void SendSetupData(Leg& leg);

  ThetaGammaZLegControl leg_control_;

  int selected_leg_ = 0;
  // ThetaGammaScheme joystick_scheme_;
};

class CalibrationLegControl : public LegControl {
 public:
  enum CalibrationMode {
    LinearizationReading,
    LinearizationValidation,
    ElectricZero
  };

  bool Process(Leg& leg, float dt) override;

  CalibrationMode GetCalibrationMode() const { return calibration_mode_; }
  void SetCalibrationMode(CalibrationMode mode);
  void SetMotorIndex(int index);
  void SendSetupData(Leg& leg);
  void SetRunning(bool running) {
    std::cout << "Set running: " << running << std::endl;
    running_ = running;
    step_ = 0;
    next_read_delta_ = -0.1f;
    // if (running_ && calibration_mode_ == CalibrationMode::ElectricZero) {
    //   std::cout << "Delta set" << std::endl;
    //   next_read_delta_ = 1.5f;
    // }
  }
  bool IsRunning() const { return running_; }

  int GetStep() const { return step_; }
  void SetStep(int step) { step_ = step; }

  uint16_t raw_angle_ = 0;
 private:
  Motor* GetMotor(Leg& leg, int index) const;
  CalibrationMode calibration_mode_ = CalibrationMode::LinearizationReading;
  int motor_index_ = 0;
  bool running_ = false;
  int step_ = 0;
  float next_read_delta_ = 0.f;
  int zero_angle_ = 0;
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
  Controller(const Legs& legs, const std::vector<BLDCDriverBoard*>& controllers,
             std::shared_ptr<StoppedBehavior> stopped_behavior,
             std::shared_ptr<EStoppedBehavior> estopped_behavior);

  // void ProcessInput(const ControlEvent& event);

  void Update(float dt);
  inline const Legs& GetLegs() const { return legs_; }
  inline Legs& GetLegs() { return legs_; }

  inline const std::vector<BLDCDriverBoard*>& GetControllers() const {
    return controllers_;
  }
  inline std::vector<BLDCDriverBoard*>& GetControllers() {
    return controllers_;
  }

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

 private:
  void DistributeEvents(const std::deque<ControlEvent>& events);

  Legs legs_;
  std::vector<BLDCDriverBoard*> controllers_;
  // Behavior* current_behavior_ = {};
  //  std::vector<std::shared_ptr<EventFactory>> control_sources_ = {};

  std::shared_ptr<StoppedBehavior> stopped_behavior_;
  std::shared_ptr<EStoppedBehavior> estopped_behavior_;
  std::vector<std::shared_ptr<EventNode>> nodes_;
  std::vector<EventNode*> tickables_;
  std::unordered_multimap<EventId, EventNode*> subscriptions_;

  // DriveThetaGammaBehavior theta_gamma_behavior_;
  // DriveXYZBehavior xyz_behavior_;
};