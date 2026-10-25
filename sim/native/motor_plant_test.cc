#include "motor_plant.h"
#include "gtest/gtest.h"
#include <limits>

TEST(MotorPlant, FirmwareVoltageScalingResistanceAndPowerBalance) {
  simulation::MotorParameters m;
  // 10 ohm line-line -> 5 ohm equivalent phase. A 4 V wire command
  // produces 2 V peak phase, 0.4 A peak phase, and 0.0420285 Nm shaft torque.
  EXPECT_NEAR(m.Torque(-4,0,0.2941995,true),0.0420285,1e-12);
  EXPECT_NEAR(m.Torque(4,0,0.2941995,true),-0.0420285,1e-12);
  EXPECT_NEAR(m.CopperLoss(0.0420285),1.2,1e-12);
  EXPECT_NEAR(m.Torque(-100,0,0.2941995,true),0.2941995,1e-12); // 28 V command cap
  EXPECT_NEAR(m.Torque(-28,0,1,true),3000 * 0.0000980665,1e-12);
  EXPECT_NEAR(m.Torque(-14,0,1,true),1500 * 0.0000980665,1e-12);
  EXPECT_NEAR(m.Torque(-28,0,0.02,true),0.02,1e-12); // shaft torque cap
  EXPECT_LT(m.Torque(0,1,1,true),0);
  EXPECT_EQ(m.Torque(28,100,1,false),0);
  const double speed = 20;
  const double torque = m.Torque(-8,speed,1,true);
  const double current = torque/m.TorqueConstant();
  EXPECT_NEAR(1.5*4*current, torque*speed + m.CopperLoss(torque),1e-12);
  m.resistance_line_to_line *= 2;
  EXPECT_NEAR(m.Torque(-4,0,1,true),0.02101425,1e-12);
  EXPECT_NEAR(m.CopperLoss(0.02101425),0.6,1e-12);
}

TEST(MotorPlant, EncodersAndInvalidParameters) {
  simulation::MotorParameters m;
  EXPECT_EQ(m.Encoder(0),0);
  EXPECT_EQ(m.Encoder(-std::acos(-1.0)),2048);
  EXPECT_EQ(m.Encoder(2*std::acos(-1.0)),0);
  m.zero_counts = 1234; EXPECT_EQ(m.Encoder(0),1234);
  m.phase_back_emf = 0; EXPECT_THROW(m.Validate(),std::invalid_argument);
  m.phase_back_emf = 0.0700475;
  m.resistance_line_to_line = 0; EXPECT_THROW(m.Validate(),std::invalid_argument);
  m.resistance_line_to_line = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(m.Validate(),std::invalid_argument);
}

TEST(MotorTemperature, CopperHeatingCoolingAndStepConvergence) {
  simulation::ThermalParameters p;
  p.heat_capacity_j_per_k = 10;
  p.thermal_resistance_k_per_w = 3;
  simulation::MotorTemperature coarse(p), fine(p);
  coarse.Step(8,30);
  for (int i = 0; i < 3000; ++i) fine.Step(8,0.01);
  EXPECT_NEAR(coarse.Celsius(),20 + 24*(1-std::exp(-1)),1e-12);
  EXPECT_NEAR(fine.Celsius(),coarse.Celsius(),1e-10);
  const double hot = coarse.Celsius();
  coarse.Step(0,30);
  EXPECT_NEAR(coarse.Celsius(),20+(hot-20)*std::exp(-1),1e-12);
  coarse.Step(0,30000);
  EXPECT_NEAR(coarse.Celsius(),20,1e-12);
  fine.Reset(); EXPECT_EQ(fine.Celsius(),20);
}

TEST(MotorTemperature, SaturatedTorqueAndDisabledBridgeDetermineLoss) {
  simulation::MotorParameters motor;
  simulation::MotorTemperature a, b;
  a.Step(motor.CopperLoss(motor.Torque(28,0,0.02,true)),1);
  b.Step(motor.CopperLoss(motor.Torque(-280,0,0.02,true)),1);
  EXPECT_EQ(a.Celsius(),b.Celsius());
  EXPECT_GT(a.Celsius(),20);
  const auto hot = a.Celsius();
  a.Step(motor.CopperLoss(motor.Torque(28,100,0.02,false)),1);
  EXPECT_LT(a.Celsius(),hot);
}

TEST(MotorTemperature, QuantizationAndValidation) {
  simulation::ThermalParameters p;
  p.initial_c = 31.6;
  simulation::MotorTemperature sensor(p);
  EXPECT_EQ(sensor.Sensor(),32);
  p.initial_c = -10; EXPECT_EQ(simulation::MotorTemperature(p).Sensor(),0);
  p.initial_c = 300; EXPECT_EQ(simulation::MotorTemperature(p).Sensor(),255);
  p.heat_capacity_j_per_k = -1; EXPECT_THROW(p.Validate(),std::invalid_argument);
  p.heat_capacity_j_per_k = 1;
  p.thermal_resistance_k_per_w = 0; EXPECT_THROW(p.Validate(),std::invalid_argument);
  p.thermal_resistance_k_per_w = 1;
  p.ambient_c = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(p.Validate(),std::invalid_argument);
  EXPECT_THROW(sensor.Step(-1,1),std::invalid_argument);
  EXPECT_THROW(sensor.Step(1,-1),std::invalid_argument);
}
