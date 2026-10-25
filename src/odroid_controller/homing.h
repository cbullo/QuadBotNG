#pragma once

#include <array>
#include <string>
#include "pid_utils.h"

// Encoder-only homing. Coordinates retain the three-motor differential. A
// repeatable stall is evidence of an obstruction, not a contact sensor: the
// configured stops and floor reference must still match the real mechanism.
class LegHoming {
 public:
  using Pose = std::array<float, 3>;  // theta, gamma, z
  enum class Stage {
    Waiting, SeekGamma, BackoffGamma, RepeatGamma,
    SeekZ, BackoffZ, RepeatZ, MoveZ, MoveGamma,
    SeekFloor, BackoffFloor, RepeatFloor, Park, Done, Failed, Standing
  };
  struct Config {
    float gamma_direction = 1, theta_direction = 1, z_gamma_direction = 1;
    float z_direction = 1;
    float gamma_stop = 0, z_stop = 0, z_reference = 0, floor_theta = 0;
    // Physical coordinate at which the stored Z encoder reading was taken.
    float z_reference_position = 0;
    std::array<PIDParams, 3> pid;
  };
  void Capture(const Config& config, const Pose& raw);
  void Start();
  void SetStandingTarget(const Pose& calibrated);
  // Returns I/O/Z voltages before encoder-direction correction. No simulator
  // contact, chassis pose or true joint angle enters this state machine.
  Pose Step(const Pose& raw, float dt);
  Stage GetStage() const { return stage_; }
  const char* StageName() const;
  const std::string& Fault() const { return fault_; }
  const Pose& Correction() const { return correction_; }
  const Pose& Target() const { return target_; }
  const Pose& Effort() const { return applied_effort_; }

 private:
  void Enter(Stage stage);
  void Fail(const char* reason);
  int ProbeAxis() const;
  bool Seeking() const;
  bool FreeTheta() const;
  bool Repeating() const;
  bool Settled(int except = -1) const;
  void AcceptStop(int axis);
  Pose Control(float dt);

  Config config_;
  Stage stage_ = Stage::Waiting;
  Pose q_{}, previous_{}, velocity_{}, target_{}, ramp_{}, integral_{};
  Pose correction_{}, applied_effort_{};
  float elapsed_ = 0, total_ = 0, stable_time_ = 0, settle_time_ = 0;
  bool probe_started_ = false;
  float start_ = 0, first_contact_ = 0, stable_position_ = 0;
  std::string fault_;
};
