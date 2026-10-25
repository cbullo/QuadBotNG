#include "homing.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr float kBackoff = 0.10f;
constexpr float kRepeatTolerance = 0.025f;
constexpr float kPositionTolerance = 0.045f;
// Preserve the original initializer's motor command ceiling. Theta floor
// probing has a separate 2.5 V generalized cap (1.25 V common-mode per I/O motor).
constexpr float kMotorVoltageLimit = 24.f;
// Voltage gains for the 3000 gf cm / 28 V motor approximation. The former
// weak-motor gains excite the coupled leg after ground contact.
constexpr float kGainScale = 0.25f;
constexpr float kPi = 3.14159265358979323846f;
}

void LegHoming::Capture(const Config& config, const Pose& raw) {
  *this = LegHoming();
  config_ = config;
  q_ = previous_ = target_ = ramp_ = raw;
}
void LegHoming::Start() {
  if (stage_ == Stage::Waiting) Enter(Stage::SeekGamma);
}
void LegHoming::SetStandingTarget(const Pose& calibrated) {
  if (stage_ != Stage::Done && stage_ != Stage::Standing)
    throw std::logic_error("standing requires completed homing");
  for (float q : calibrated) if (!std::isfinite(q))
    throw std::invalid_argument("non-finite standing target");
  for (int i = 0; i < 3; ++i) target_[i] = calibrated[i] - correction_[i];
  stage_ = Stage::Standing;
}
const char* LegHoming::StageName() const {
  switch (stage_) {
    case Stage::Waiting: return "waiting";
    case Stage::SeekGamma: return "seek_gamma_stop";
    case Stage::BackoffGamma: return "backoff_gamma";
    case Stage::RepeatGamma: return "repeat_gamma_stop";
    case Stage::SeekZ: return "seek_z_stop";
    case Stage::BackoffZ: return "backoff_z";
    case Stage::RepeatZ: return "repeat_z_stop";
    case Stage::MoveZ: return "move_to_safe_z";
    case Stage::MoveGamma: return "move_to_gamma_zero";
    case Stage::SeekFloor: return "seek_floor";
    case Stage::BackoffFloor: return "backoff_floor";
    case Stage::RepeatFloor: return "repeat_floor";
    case Stage::Park: return "park";
    case Stage::Done: return "done";
    case Stage::Failed: return "failed";
    case Stage::Standing: return "stand";
  }
  return "unknown";
}
int LegHoming::ProbeAxis() const {
  switch (stage_) {
    case Stage::SeekGamma: case Stage::RepeatGamma: case Stage::BackoffGamma: return 1;
    case Stage::SeekZ: case Stage::RepeatZ: case Stage::BackoffZ: case Stage::MoveZ: return 2;
    case Stage::MoveGamma: return 1;
    default: return 0;
  }
}
bool LegHoming::Seeking() const {
  return stage_ == Stage::SeekGamma || stage_ == Stage::RepeatGamma ||
         stage_ == Stage::SeekZ || stage_ == Stage::RepeatZ ||
         stage_ == Stage::SeekFloor || stage_ == Stage::RepeatFloor;
}
bool LegHoming::FreeTheta() const {
  return stage_ >= Stage::SeekGamma && stage_ <= Stage::MoveZ;
}
bool LegHoming::Repeating() const {
  return stage_ == Stage::RepeatGamma || stage_ == Stage::RepeatZ || stage_ == Stage::RepeatFloor;
}
void LegHoming::Enter(Stage stage) {
  const int old_axis = ProbeAxis();
  const bool was_seeking = Seeking();
  stage_ = stage;
  elapsed_ = stable_time_ = settle_time_ = 0;
  probe_started_ = false;
  start_ = stable_position_ = q_[ProbeAxis()];
  if (stage != Stage::Done) {
    ramp_[ProbeAxis()] = q_[ProbeAxis()];
    integral_[ProbeAxis()] = 0;
    if (was_seeking) integral_[old_axis] = 0;
  }
}
void LegHoming::Fail(const char* reason) {
  fault_ = std::string(StageName()) + ": " + reason;
  fault_ += " (theta/gamma/z errors=" + std::to_string(target_[0] - q_[0]) + "," +
      std::to_string(target_[1] - q_[1]) + "," + std::to_string(target_[2] - q_[2]) + ")";
  stage_ = Stage::Failed;
  applied_effort_ = {};
}
bool LegHoming::Settled(int except) const {
  const float tolerance = stage_ >= Stage::MoveGamma ? 0.015f : kPositionTolerance;
  for (int i = 0; i < 3; ++i) if (i != except && !(i == 0 && FreeTheta()) &&
      (std::abs(q_[i] - target_[i]) > (i == 0 ? kPositionTolerance : tolerance) || std::abs(velocity_[i]) > 0.08f))
    return false;
  return true;
}

LegHoming::Pose LegHoming::Control(float dt) {
  const Pose saved_integral = integral_;
  Pose effort{};
  // Deliberately free theta while folding and finding Z, as in the original
  // recovery sequence. This lets contact redirect the leg around the chassis.
  // Other coordinates and waiting legs are actively held.
  const Pose limits = stage_ == Stage::Standing ? Pose{48.f, 28.f, 28.f} :
      Pose{Seeking() && ProbeAxis() == 0 ? 2.5f : 16.f, 14.f, 12.f};
  for (int i = 0; i < 3; ++i) {
    if (i == 0 && FreeTheta()) {
      target_[i] = ramp_[i] = q_[i];
      integral_[i] = 0;
      // A folded upper link can rest on the floor while Z returns. A weak
      // lift in the direction opposite the later floor sweep unloads it;
      // position remains free rather than pinning it against that contact.
      if (stage_ == Stage::MoveZ)
        effort[0] = config_.theta_direction * std::clamp(
            1.5f - 1.f * config_.theta_direction * velocity_[0], 0.f, 1.5f);
      continue;
    }
    ramp_[i] += std::clamp(target_[i] - ramp_[i], -1.5f * dt, 1.5f * dt);
    const float error = ramp_[i] - q_[i];
    const auto& p = config_.pid[i];
    const Pose minimum_i{50.f, 100.f, 100.f};
    const float candidate_i = std::clamp(integral_[i] + std::max(p.i, minimum_i[i]) * error * dt, -limits[i], limits[i]);
    const Pose minimum_p{160.f, 250.f, 180.f};
    const Pose minimum_d{0.5f, 0.5f, 0.5f};
    const float pd = kGainScale * (std::max(p.p, minimum_p[i]) * error -
                     std::max(p.d, minimum_d[i]) * velocity_[i]);
    // Conditional integration prevents windup against a stop or voltage cap.
    if (std::abs(pd + candidate_i) <= limits[i] || error * (pd + candidate_i) < 0)
      integral_[i] = candidate_i;
    effort[i] = p.dir * std::clamp(pd + integral_[i], -limits[i], limits[i]);
  }
  Pose volts{0.5f * effort[0] + effort[1],
             0.5f * effort[0] - effort[1],
             effort[2] + config_.z_gamma_direction * (4.f / 3.f) * effort[1]};
  float peak = kMotorVoltageLimit;
  for (float v : volts) peak = std::max(peak, std::abs(v));
  const float scale = kMotorVoltageLimit / peak;
  // Scale the coupled vector together rather than clipping motors separately.
  // Stop evidence below uses the delivered command, not an unbounded PID sum.
  for (int i = 0; i < 3; ++i) {
    volts[i] *= scale;
    applied_effort_[i] = effort[i] * scale;
  }
  if (scale < 1) integral_ = saved_integral;
  return volts;
}

void LegHoming::AcceptStop(int axis) {
  const float contact = q_[axis];
  const float direction = axis == 1 ? -config_.gamma_direction :
                          axis == 2 ? config_.z_direction : -config_.theta_direction;
  if (!Repeating()) {
    first_contact_ = contact;
    target_[axis] = contact - direction * kBackoff;
    Enter(axis == 1 ? Stage::BackoffGamma : axis == 2 ? Stage::BackoffZ : Stage::BackoffFloor);
    return;
  }
  if (direction * (contact - start_) < kBackoff * 0.5f) {
    Fail("no movement after backoff");
    return;
  }
  if (std::abs(contact - first_contact_) > kRepeatTolerance) {
    Fail("contact did not repeat within 0.025 rad");
    return;
  }
  // Changes remain provisional until all four legs have completed. Raw encoder
  // coordinates continue driving the hold, so committing offsets cannot jump it.
  const float measured = 0.5f * (contact + first_contact_);
  if (axis == 1) {
    correction_[1] = config_.gamma_stop - measured;
    target_[1] = contact - direction * kBackoff;
    Enter(Stage::SeekZ);
  } else if (axis == 2) {
    const float period = 2.f * kPi / 9.f;
    const float offset_phase = config_.z_reference - config_.z_reference_position;
    const float offset = offset_phase + period *
        std::round((measured - config_.z_stop - offset_phase) / period);
    // Z has a stored phase. Reject the wrong stop instead of forcing a turn.
    if (std::abs(measured - offset - config_.z_stop) > 0.08f) {
      Fail("Z stop disagrees with stored encoder phase");
      return;
    }
    correction_[2] = -offset;
    target_[2] = -correction_[2];
    Enter(Stage::MoveZ);
  } else {
    correction_[0] = config_.floor_theta - measured;
    target_[0] = contact - direction * 0.25f;
    Enter(Stage::Park);
  }
}

LegHoming::Pose LegHoming::Step(const Pose& raw, float dt) {
  if (stage_ == Stage::Failed) return {};
  if (dt == 0) return {};
  if (!std::isfinite(dt) || dt < 0 || dt > 0.1f) { Fail("invalid control interval"); return {}; }
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(raw[i])) { Fail("invalid encoder angle"); return {}; }
    velocity_[i] += (dt / (0.004f + dt)) * ((raw[i] - previous_[i]) / dt - velocity_[i]);
  }
  q_ = previous_ = raw;
  elapsed_ += dt;
  if (stage_ != Stage::Waiting && stage_ != Stage::Done && stage_ != Stage::Standing) {
    total_ += dt;
    if (total_ > 26.f || elapsed_ > 8.f) { Fail("time limit exceeded"); return {}; }
  }
  const int axis = ProbeAxis();
  if (Seeking()) {
    const float direction = axis == 1 ? -config_.gamma_direction : axis == 2 ? config_.z_direction : -config_.theta_direction;
    const float travel_limit = Repeating() ? 0.35f : axis == 1 ? 2.7f : axis == 2 ? 1.8f : 2.f * kPi;
    if (std::abs(q_[axis] - start_) > travel_limit) { Fail("travel limit exceeded"); return {}; }
    // Slew a bounded distance ahead of feedback; never build up a target far
    // beyond the stop. Start the probe only after the held coordinates settle.
    if (Settled(axis)) probe_started_ = true;
    bool tracking = true;
    for (int i = 0; i < 3; ++i)
      if (i != axis && !(i == 0 && FreeTheta()) && std::abs(target_[i] - q_[i]) > 0.08f)
        tracking = false;
    if (probe_started_ && tracking)
      target_[axis] += direction * (axis == 0 ? 1.2f : 1.5f) * dt;
    target_[axis] = q_[axis] + std::clamp(target_[axis] - q_[axis], -0.3f, 0.3f);
  }
  const auto volts = Control(dt);
  if (Seeking()) {
    const float direction = axis == 1 ? -config_.gamma_direction : axis == 2 ? config_.z_direction : -config_.theta_direction;
    const float minimum_effort = axis == 0 ? 1.5f : 6.f;
    const bool loaded = direction * config_.pid[axis].dir * applied_effort_[axis] >= minimum_effort &&
                        direction * (target_[axis] - q_[axis]) >= 0.12f;
    const bool quiet = std::abs(velocity_[axis]) < 0.06f &&
                       std::abs(q_[axis] - stable_position_) < 0.008f;
    if (elapsed_ > 0.25f && loaded && quiet && Settled(axis)) stable_time_ += dt;
    else { stable_time_ = 0; stable_position_ = q_[axis]; }
    if (stable_time_ >= 0.18f) AcceptStop(axis);
  } else {
    settle_time_ = Settled() ? settle_time_ + dt : 0;
    if (settle_time_ >= 0.12f) {
      switch (stage_) {
        case Stage::BackoffGamma: Enter(Stage::RepeatGamma); break;
        case Stage::BackoffZ: Enter(Stage::RepeatZ); break;
        case Stage::BackoffFloor: Enter(Stage::RepeatFloor); break;
        case Stage::MoveZ:
          target_[1] = -correction_[1]; Enter(Stage::MoveGamma); break;
        case Stage::MoveGamma: Enter(Stage::SeekFloor); break;
        case Stage::Park: Enter(Stage::Done); break;
        default: break;
      }
    }
  }
  return stage_ == Stage::Failed ? Pose{} : volts;
}
