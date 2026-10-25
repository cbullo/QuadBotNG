#include "motor_plant.h"
#include "gtest/gtest.h"
#include <limits>
TEST(MotorPlant, UnitsSignBackEmfAndDisabledBridge) {
  simulation::MotorParameters m;
  EXPECT_NEAR(m.Torque(1,0,0.2941995,true),-0.03,1e-12);
  EXPECT_NEAR(m.Torque(-1,0,0.2941995,true),0.03,1e-12);
  EXPECT_NEAR(m.Torque(100,0,0.2941995,true),-0.2941995,1e-12);
  EXPECT_LT(m.Torque(0,1,1,true),0);
  EXPECT_EQ(m.Torque(28,100,1,false),0);
  EXPECT_EQ(m.Encoder(0),0);
  EXPECT_EQ(m.Encoder(-std::acos(-1.0)),2048);
  EXPECT_EQ(m.Encoder(2*std::acos(-1.0)),0);
  m.zero_counts = 1234; EXPECT_EQ(m.Encoder(0),1234);
  m.back_emf = -1; EXPECT_THROW(m.Validate(),std::invalid_argument);
}
TEST(MotorTemperature, CopperHeatingCoolingAndStepConvergence) {
  simulation::ThermalParameters p;
  p.resistance_ohm = 2;
  p.heat_capacity_j_per_k = 10;
  p.thermal_resistance_k_per_w = 3;
  simulation::MotorTemperature coarse(p), fine(p), reverse(p);
  // gain=.5 => Kt=1 Nm/A. Torque=2 Nm => I=2 A, copper loss=8 W.
  coarse.Step(2,0.5,30);
  reverse.Step(-2,0.5,30);
  for (int i = 0; i < 3000; ++i) fine.Step(2,0.5,0.01);
  EXPECT_NEAR(coarse.Celsius(),20 + 24*(1-std::exp(-1)),1e-12);
  EXPECT_NEAR(fine.Celsius(),coarse.Celsius(),1e-10);
  EXPECT_EQ(reverse.Celsius(),coarse.Celsius());
  const double hot = coarse.Celsius();
  coarse.Step(0,0.5,30);
  EXPECT_NEAR(coarse.Celsius(),20+(hot-20)*std::exp(-1),1e-12);
  coarse.Step(0,0.5,30000);
  EXPECT_NEAR(coarse.Celsius(),20,1e-12);
  fine.Reset(); EXPECT_EQ(fine.Celsius(),20);
}
TEST(MotorTemperature, SaturatedTorqueAndDisabledBridgeDetermineLoss) {
  simulation::MotorParameters motor;
  simulation::MotorTemperature a, b;
  a.Step(motor.Torque(28,0,0.2941995,true),motor.torque_per_volt,1);
  b.Step(motor.Torque(280,0,0.2941995,true),motor.torque_per_volt,1);
  EXPECT_EQ(a.Celsius(),b.Celsius());
  EXPECT_GT(a.Celsius(),20);
  const auto hot = a.Celsius();
  a.Step(motor.Torque(28,100,0.2941995,false),motor.torque_per_volt,1);
  EXPECT_LT(a.Celsius(),hot);
}
TEST(MotorTemperature, QuantizationAndValidation) {
  simulation::ThermalParameters p;
  p.initial_c = 31.6;
  simulation::MotorTemperature sensor(p);
  EXPECT_EQ(sensor.Sensor(),32);
  p.initial_c = -10; EXPECT_EQ(simulation::MotorTemperature(p).Sensor(),0);
  p.initial_c = 300; EXPECT_EQ(simulation::MotorTemperature(p).Sensor(),255);
  p.resistance_ohm = 0; EXPECT_THROW(p.Validate(),std::invalid_argument);
  p.resistance_ohm = 1;
  p.heat_capacity_j_per_k = -1; EXPECT_THROW(p.Validate(),std::invalid_argument);
  p.heat_capacity_j_per_k = 1;
  p.thermal_resistance_k_per_w = 0; EXPECT_THROW(p.Validate(),std::invalid_argument);
  p.thermal_resistance_k_per_w = 1;
  p.ambient_c = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(p.Validate(),std::invalid_argument);
  EXPECT_THROW(sensor.Step(1,0,1),std::invalid_argument);
  EXPECT_THROW(sensor.Step(1,1,-1),std::invalid_argument);
}
