#include <gtest/gtest.h>
#include <math.h>

#include "src/odroid_controller/angle.h"

class TestMotor {
 public:
  static constexpr float kAS5600ToRadians = M_PI / 2048.0;
  static constexpr float kRadiansToAS5600 = 2048.0 / M_PI;

  void UpdateAngle(uint16_t new_angle) {
    if (first_update) {
      offset_angle_ = new_angle * kAS5600ToRadians;
      zero_angle_ = offset_angle_;
      first_update = false;
      raw_angle_ = new_angle;
      // std::cout << "Zero angle set: " << zero_angle_ << std::endl;
      // std::cout << "Accumulated angle: " << GetAccumulatedAngle() <<
      // std::endl;
      return;
    }

    auto prev_angle = offset_angle_;
    offset_angle_ = new_angle;

    // std::cout << prev_offset_angle_ << " " << offset_angle_ << std::endl;

    auto diff = new_angle - prev_angle;
    if (diff <= -2048) {
      diff += 4096;
    } else if (diff > 2048) {
      diff -= 4096;
    }


    accumulated_angle_ += diff;
    

    raw_angle_ = new_angle;

    // std::cout << "Accumulated angle: " << GetAccumulatedAngle() << std::endl;
  }

  float GetAngleZ() const { return accumulated_angle_ * kAS5600ToRadians / 9.0 - z_offset_; }

  uint16_t raw_angle_ = 0.0;
  float zero_angle_ = 0.f;
  float z_offset_ = 0.f;

  int16_t offset_angle_ = 0.0;
  //  float prev_offset_angle_ = 0.0;
  int64_t accumulated_angle_ = 0.0;
  float gear_ratio_ = 1.0;
  int direction_ = 1;
  bool first_update = true;
};

// Demonstrate some basic assertions.
TEST(AngleTests, Init) {
  TestMotor motor;
  motor.UpdateAngle(0);

  EXPECT_FALSE(motor.first_update);
}

TEST(AngleTests, Step) {
  TestMotor motor;
  motor.UpdateAngle(0);
  motor.UpdateAngle(1);

  EXPECT_FLOAT_EQ(motor.GetAngleZ(), 0.001533980789 / 9.0);
}

TEST(AngleTests, MinusStep) {
  TestMotor motor;
  motor.UpdateAngle(0);
  motor.UpdateAngle(4095);

  EXPECT_NEAR(motor.GetAngleZ(), -0.00153398078788564122971808758949 / 9.0,
              0.00001);
}

TEST(AngleTests, MinusPlusStep) {
  TestMotor motor;
  motor.UpdateAngle(0);
  motor.UpdateAngle(4095);
  motor.UpdateAngle(1);

  EXPECT_NEAR(motor.GetAngleZ(), 0.001533980789 / 9.0, 0.00001);
}

TEST(AngleTests, PlusCycleStep) {
  TestMotor motor;
  for (int c = 0; c < 9; ++c) {
    for (int i = 0; i <= 4095; ++i) {
      std::cout << i << " " << motor.offset_angle_ << " " << motor.accumulated_angle_ << std::endl;
      motor.UpdateAngle(i);
    }
  }
  motor.UpdateAngle(0);

  EXPECT_FLOAT_EQ(motor.GetAngleZ(), 2.0 * M_PI);
}

TEST(AngleTests, MinusCycleStep) {
  TestMotor motor;
  motor.UpdateAngle(0);
  for (int c = 0; c < 9; ++c) {
    for (int i = 4095; i >= 0; --i) {
      std::cout << i << " " << motor.offset_angle_ << " " << motor.accumulated_angle_ << std::endl;
      motor.UpdateAngle(i);
    }
  }
  motor.UpdateAngle(0);

  EXPECT_FLOAT_EQ(motor.GetAngleZ(), -2.0 * M_PI);
}

TEST(AngleTests, PlusCycleLargeStep) {
  TestMotor motor;
  motor.UpdateAngle(0);
  for (int c = 0; c < 9; ++c) {
    for (int i = 0; i <= 4095; i+=10) {
      std::cout << i << " " << motor.offset_angle_ << " " << motor.accumulated_angle_ << std::endl;
      motor.UpdateAngle(i);
    }
  }
  motor.UpdateAngle(0);

  EXPECT_FLOAT_EQ(motor.GetAngleZ(), 2.0 * M_PI);
}

TEST(AngleTests, MinusCycleLargeStep) {
  TestMotor motor;
  motor.UpdateAngle(0);
  for (int c = 0; c < 9; ++c) {
    for (int i = 4095; i >= 0; i-=10) {
      std::cout << i << " " << motor.offset_angle_ << " " << motor.accumulated_angle_ << std::endl;
      motor.UpdateAngle(i);
    }
  }
  motor.UpdateAngle(0);

  EXPECT_FLOAT_EQ(motor.GetAngleZ(), -2.0 * M_PI);
}

TEST(AngleTests, MinusTwoCyclesLargeStep) {
  TestMotor motor;
  motor.UpdateAngle(0);
  for (int c = 0; c < 18; ++c) {
    for (int i = 4095; i >= 0; i-=10) {
      std::cout << i << " " << motor.offset_angle_ << " " << motor.accumulated_angle_ << std::endl;
      motor.UpdateAngle(i);
    }
  }
  motor.UpdateAngle(0);

  EXPECT_FLOAT_EQ(motor.GetAngleZ(), -4.0 * M_PI);
}