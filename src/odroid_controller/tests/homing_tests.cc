#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "gtest/gtest.h"
#include "src/odroid_controller/homing.h"

namespace {
using Stage = LegHoming::Stage;
struct Rig {
  LegHoming homing;
  LegHoming::Config config;
  LegHoming::Pose q{0, 0, 0}, offset{0.7f, 0.3f, -0.2f};
  static constexpr float dt = 0.005f;
  explicit Rig(float direction = 1, float z_direction = 1) {
    config.gamma_direction = config.theta_direction = direction;
    config.z_gamma_direction = -direction * z_direction;
    config.gamma_stop = -direction * 1.4835f;
    config.z_direction = z_direction;
    config.z_stop = z_direction > 0 ? 1.17f : -0.2f;
    config.z_reference_position = 1.17f;
    config.z_reference = std::fmod(offset[2] + config.z_reference_position,
                                  2.f * std::acos(-1.f) / 9.f);
    config.floor_theta = -direction * 1.039f;
    config.pid = {PIDParams{50, 0.5, 25, 1}, PIDParams{20, 0.5, 30, 1}, PIDParams{50, 0.5, 25, -1}};
    q = offset;
    homing.Capture(config, q);
    homing.Start();
  }
  void Step(bool move = true, float second_floor_shift = 0) {
    const auto volts = homing.Step(q, dt);
    for (float v : volts) EXPECT_LE(std::abs(v), 24.0001f);
    if (!move) return;
    // Deliberately simple overdamped test fixture, not a robot physics model.
    // Driven by delivered coupled commands with independent physical stops.
    LegHoming::Pose effort{volts[0] + volts[1], (volts[0] - volts[1]) / 2, 0};
    effort[2] = volts[2] - config.z_gamma_direction * (4.f/3.f) * effort[1];
    for (int i = 0; i < 3; ++i)
      q[i] += std::clamp(config.pid[i].dir * effort[i] * 0.3f, -1.f, 1.f) * dt;
    const float d = config.gamma_direction;
    q[1] = d * std::max(d * q[1], d * (config.gamma_stop + offset[1]));
    q[2] = config.z_direction * std::min(config.z_direction * q[2],
        config.z_direction * (config.z_stop + offset[2]));
    const float floor = config.floor_theta + offset[0] +
        (homing.GetStage() == Stage::RepeatFloor ? d * second_floor_shift : 0);
    q[0] = d * std::max(d * q[0], d * floor);
  }
  void Until(Stage stage, float max_seconds = 100, float shift = 0) {
    for (int n = 0; n < max_seconds / dt && homing.GetStage() != stage &&
         homing.GetStage() != Stage::Failed; ++n) Step(true, shift);
  }
};

TEST(Homing, RepeatedStopsCompleteForBothLegDirections) {
  for (float direction : {-1.f, 1.f}) for (float z_direction : {-1.f, 1.f}) {
    Rig rig(direction, z_direction);
    rig.Until(Stage::Done);
    ASSERT_EQ(rig.homing.GetStage(), Stage::Done) << rig.homing.StageName() << ": " << rig.homing.Fault();
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(rig.homing.Correction()[i], -rig.offset[i], 0.015);
    const auto target = rig.homing.Target();
    for (int i = 0; i < 400; ++i) rig.Step();
    EXPECT_EQ(rig.homing.GetStage(), Stage::Done);
    EXPECT_EQ(target, rig.homing.Target());  // No automatic stand-up.
  }
}
TEST(Homing, EitherZSearchRejectsAnIncorrectReferencePhase) {
  for (float direction : {-1.f, 1.f}) {
    Rig rig(1, direction);
    rig.config.z_reference += 0.15f;
    rig.homing.Capture(rig.config, rig.q);
    rig.homing.Start();
    rig.Until(Stage::Done);
    EXPECT_EQ(rig.homing.GetStage(), Stage::Failed);
    EXPECT_NE(rig.homing.Fault().find("stored encoder phase"), std::string::npos);
  }
}
TEST(Homing, JamCannotPassBackoffAndNeverLoops) {
  Rig rig;
  for (int i = 0; i < 6000; ++i) rig.Step(false);
  EXPECT_EQ(rig.homing.GetStage(), Stage::Failed);
  EXPECT_NE(rig.homing.Fault().find("backoff_gamma"), std::string::npos);
  EXPECT_EQ(rig.homing.Correction(), (LegHoming::Pose{}));
  EXPECT_EQ(rig.homing.Step(rig.q, Rig::dt), (LegHoming::Pose{}));
}
TEST(Homing, DifferentSecondFloorContactIsRejected) {
  Rig rig;
  rig.Until(Stage::RepeatFloor);
  ASSERT_EQ(rig.homing.GetStage(), Stage::RepeatFloor) << rig.homing.Fault();
  rig.Until(Stage::Done, 30, -0.06f);
  EXPECT_EQ(rig.homing.GetStage(), Stage::Failed);
  EXPECT_NE(rig.homing.Fault().find("did not repeat"), std::string::npos);
  EXPECT_EQ(rig.homing.Correction()[0], 0);
}
TEST(Homing, FloorProbeWaitsForFinalGammaAndZTargets) {
  Rig rig;
  rig.Until(Stage::MoveGamma);
  ASSERT_EQ(rig.homing.GetStage(), Stage::MoveGamma) << rig.homing.Fault();
  for (int n = 0; n < 100; ++n) rig.Step(false);
  EXPECT_EQ(rig.homing.GetStage(), Stage::MoveGamma);
  rig.Until(Stage::SeekFloor);
  ASSERT_EQ(rig.homing.GetStage(), Stage::SeekFloor) << rig.homing.Fault();
  EXPECT_NEAR(rig.q[1] - rig.offset[1], 0, 0.045);
  EXPECT_NEAR(rig.q[2] - rig.offset[2], 0, 0.045);
}
TEST(Homing, FreeSpinningLegIsBounded) {
  Rig rig;
  rig.Until(Stage::SeekFloor);
  ASSERT_EQ(rig.homing.GetStage(), Stage::SeekFloor) << rig.homing.Fault();
  for (int i = 0; i < 6000 && rig.homing.GetStage() != Stage::Failed; ++i) {
    rig.q[0] -= 1.f * Rig::dt;
    rig.homing.Step(rig.q, Rig::dt);
  }
  EXPECT_EQ(rig.homing.GetStage(), Stage::Failed);
  EXPECT_NE(rig.homing.Fault().find("travel limit"), std::string::npos);
}
TEST(Homing, ThetaRemainsFreeDuringFoldAndZSearch) {
  Rig rig;
  for (int n = 0; n < 50; ++n) {
    rig.q[0] += 0.003f;  // contact is allowed to redirect the unreferenced swing
    const auto volts = rig.homing.Step(rig.q, Rig::dt);
    EXPECT_FLOAT_EQ(volts[0] + volts[1], 0);
    EXPECT_FLOAT_EQ(rig.homing.Target()[0], rig.q[0]);
  }
}
TEST(Homing, StandingRequiresReferenceAndUsesCalibratedTargets) {
  Rig rig;
  EXPECT_THROW(rig.homing.SetStandingTarget({0, 0, 0}), std::logic_error);
  rig.Until(Stage::Done);
  ASSERT_EQ(rig.homing.GetStage(), Stage::Done) << rig.homing.Fault();
  const LegHoming::Pose target{0.2f, 0.4f, 0.1f};
  rig.homing.SetStandingTarget(target);
  EXPECT_EQ(rig.homing.GetStage(), Stage::Standing);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(rig.homing.Target()[i] + rig.homing.Correction()[i], target[i], 1e-6);
  auto invalid = target;
  invalid[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(rig.homing.SetStandingTarget(invalid), std::invalid_argument);
}
TEST(Homing, InvalidFeedbackFailsAndCaptureResetsFault) {
  Rig rig;
  auto invalid = rig.q;
  invalid[1] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(rig.homing.Step(invalid, Rig::dt), (LegHoming::Pose{}));
  EXPECT_EQ(rig.homing.GetStage(), Stage::Failed);
  rig.homing.Capture(rig.config, rig.q);
  EXPECT_TRUE(rig.homing.Fault().empty());
  EXPECT_EQ(rig.homing.GetStage(), Stage::Waiting);
  EXPECT_EQ(rig.homing.Correction(), (LegHoming::Pose{}));
}
}  // namespace
