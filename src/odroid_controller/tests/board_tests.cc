#include "gtest/gtest.h"
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "src/odroid_controller/boards/emulated_board.h"
#include "src/odroid_controller/motor.h"
#include "src/odroid_controller/robot_system.h"

namespace {
YAML::Node MotorConfig() {
  return YAML::Load("encoder_direction: 1\ngear_ratio_inv: 9\nlinearization_offset: 3\n"
                    "linearization_coeffs: [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]\n"
                    "electric_zero_offset: 264\nswap_wires: 1\n");
}
struct BoardTest : ::testing::Test {
  std::shared_ptr<boards::ManualClock> clock = std::make_shared<boards::ManualClock>();
  std::shared_ptr<boards::MemoryLink> link = std::make_shared<boards::MemoryLink>();
  boards::EmulatedBoard board{std::make_unique<boards::MemoryTransport>(link,1,clock),clock,link};
  BLDCDriverBoard client{std::make_unique<boards::MemoryTransport>(link,0,clock),clock};
  Motor m0{&client,0}, m1{&client,1};
  uint16_t temperature0 = 20, temperature1 = 20;
  void SetUp() override {
    link->max_chunk = 1; link->capacity = 8;
    m0.UpdateConfig(MotorConfig()); m1.UpdateConfig(MotorConfig());
    client.SetMotor(0,&m0); client.SetMotor(1,&m1);
    ASSERT_TRUE(client.Connect());
  }
  void Advance(int64_t us) {
    const auto end = clock->now + us;
    while (clock->now < end) {
      board.SetSensors(0,4090,temperature0); board.SetSensors(1,1234,temperature1);
      client.Tick(); board.Pump(); client.Tick();
      clock->now += 100;
    }
  }
};
TEST(Protocol, ExplicitSignedBytesAndWidths) {
  boards::Bytes b{uint8_t(CMD_MOTOR_VOLTAGE | MOTOR_INDEX_BIT)};
  boards::Append(b,int16_t(-512));
  EXPECT_EQ(b,(boards::Bytes{0x4a,0x00,0xfe}));
  EXPECT_EQ(boards::I16(b.data()+1),-512);
  EXPECT_NO_THROW(boards::ValidateCommand(b));
  EXPECT_THROW(boards::ValidateCommand(boards::Bytes{CMD_LIMITS,SCMD_LIM_VOLT,1}),std::invalid_argument);
  EXPECT_THROW(boards::ValidateCommand(boards::Bytes{uint8_t(CMD_MOTOR_VOLTAGE|GET_CMD_BIT)}),std::invalid_argument);
  EXPECT_EQ(boards::MessageSize(boards::Bytes{0x4b,2,0}),3u);
}
TEST(Protocol, VoltageLimitWireFormat) {
  // SET: command + subcommand + int16. GET: command + subcommand only.
  // Reply: command/type header + int16 + sequence byte.
  const boards::Bytes set{0x43,0x01,0x34,0x12};
  const boards::Bytes get{0xc3,0x01};
  const boards::Bytes reply{0x43,0x34,0x12,0x00};
  EXPECT_EQ(GetArgumentLength(CMD_LIMITS),2);
  EXPECT_EQ(boards::CommandSize(set[0]),set.size());
  EXPECT_EQ(boards::CommandSize(get[0]),get.size());
  EXPECT_EQ(boards::MessageSize(reply),reply.size());
  EXPECT_NO_THROW(boards::ValidateCommand(set));
  EXPECT_NO_THROW(boards::ValidateCommand(get));
  EXPECT_EQ(boards::I16(reply.data()+1),0x1234);
  EXPECT_THROW(boards::ValidateCommand(boards::Bytes{0x03,0x01,0x34}),std::invalid_argument);
  EXPECT_THROW(boards::ValidateCommand(boards::Bytes{0x03,SCMD_LIM_CURR,0,0}),std::invalid_argument);
  EXPECT_THROW(boards::ValidateCommand(boards::Bytes{0x83,SCMD_LIM_VEL}),std::invalid_argument);
}
TEST_F(BoardTest, FragmentedVoltageLimitsKeepFollowingCommandsAligned) {
  Advance(3500000);
  ASSERT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(board.channel(0).voltage_limit,12*512-1);
  EXPECT_EQ(board.channel(1).voltage_limit,12*512-1);
  client.SendSetCommandAndSub(0,CMD_LIMITS,SCMD_LIM_VOLT,int16_t(0x1234));
  client.SendSetCommandAndSub(1,CMD_LIMITS,SCMD_LIM_VOLT,int16_t(0x0567));
  client.SendSetCommand(0,CMD_STATE,uint8_t(3));
  client.SendSetCommand(1,CMD_MOTOR_VOLTAGE,int16_t(-1024));
  int replies = 0;
  client.Query(0,CMD_LIMITS,[&](const auto& bytes) {
    EXPECT_EQ(bytes,(boards::Bytes{0x34,0x12})); ++replies;
  },SCMD_LIM_VOLT);
  client.Query(1,CMD_LIMITS,[&](const auto& bytes) {
    EXPECT_EQ(bytes,(boards::Bytes{0x67,0x05})); ++replies;
  },SCMD_LIM_VOLT);
  client.Query(0,CMD_STATE,[&](const auto& bytes) {
    EXPECT_EQ(bytes,(boards::Bytes{3})); ++replies;
  });
  Advance(100000);
  EXPECT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(replies,3);
  EXPECT_EQ(board.channel(0).voltage_limit,0x1234);
  EXPECT_EQ(board.channel(1).voltage_limit,0x0567);
  EXPECT_EQ(board.Voltage(1),-2);
  // Signed payload bits are retained exactly, as in the firmware register.
  client.SendSetCommandAndSub(1,CMD_LIMITS,SCMD_LIM_VOLT,int16_t(-512));
  client.Query(1,CMD_LIMITS,[&](const auto& bytes) {
    EXPECT_EQ(bytes,(boards::Bytes{0x00,0xfe}));
    EXPECT_EQ(boards::I16(bytes.data()),-512); ++replies;
  },SCMD_LIM_VOLT);
  Advance(100000);
  EXPECT_EQ(replies,4);
  client.ResetBoard(); Advance(3500000);
  ASSERT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(board.channel(0).voltage_limit,12*512-1);
  EXPECT_EQ(board.channel(1).voltage_limit,12*512-1);
}
TEST(ProductionMotor, InitialReadingWrapAndReset) {
  BLDCDriverBoard client;
  Motor m(&client,0);
  auto sample = [&](uint16_t count) { m.SetCurrentRawAngle(count); m.Update(); };
  sample(4090);
  EXPECT_NEAR(m.GetAccumulatedAngle(),4090*Motor::kAS5600ToRadians,1e-12);
  sample(6);
  EXPECT_NEAR(m.GetAccumulatedAngle(),4102*Motor::kAS5600ToRadians,1e-12);
  sample(4090);
  EXPECT_NEAR(m.GetAccumulatedAngle(),4090*Motor::kAS5600ToRadians,1e-12);
  m.ResetFeedback(); sample(1234);
  EXPECT_NEAR(m.GetAccumulatedAngle(),1234*Motor::kAS5600ToRadians,1e-12);
}
TEST_F(BoardTest, FragmentedStartupConfigurationAndTelemetry) {
  Advance(3500000);
  ASSERT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(board.state(),boards::State::Stopped);
  EXPECT_EQ(board.channel(0).electrical_zero,264);
  EXPECT_EQ(board.channel(1).linearization[15],15);
  EXPECT_EQ(board.channel(0).linearization_offset,3);
  EXPECT_TRUE(board.channel(1).wires_swapped);
  EXPECT_EQ(m0.GetRawAngle(),4090); EXPECT_EQ(m1.GetRawAngle(),1234);
  EXPECT_EQ(m0.GetTemperature(),20);
}
TEST_F(BoardTest, RunStopAndExactlyOnceReplies) {
  Advance(3500000);
  client.SendSetCommand(0,CMD_STATE,uint8_t(3));
  client.SendSetCommand(0,CMD_MOTOR_VOLTAGE,int16_t(512));
  client.SendSetCommand(1,CMD_MOTOR_VOLTAGE,int16_t(-1024));
  Advance(100000);
  EXPECT_EQ(board.Voltage(0),1); EXPECT_EQ(board.Voltage(1),-2);
  int replies = 0;
  client.Query(0,CMD_STATE,[&](const auto& b) { EXPECT_EQ(b,(boards::Bytes{3})); ++replies; });
  client.Query(1,CMD_SENSOR,[&](const auto& b) { EXPECT_EQ(boards::U16(b.data()),264); ++replies; },SCMD_SENS_ELEC_OFFSET);
  Advance(100000);
  EXPECT_EQ(replies,2);
  Advance(100000); EXPECT_EQ(replies,2);
  client.SendSetCommand(0,CMD_STATE,uint8_t(2)); Advance(100000);
  EXPECT_EQ(board.Voltage(0),0); EXPECT_EQ(board.Voltage(1),0);
}
TEST_F(BoardTest, ResetResendsSetupAndEstopLatches) {
  Advance(3500000);
  client.SendSetCommand(0,CMD_STATE,uint8_t(3)); Advance(100000);
  client.SetErrorState(true); Advance(3500000);
  EXPECT_FALSE(client.IsReady()); EXPECT_EQ(board.state(),boards::State::PreInit);
  client.SetErrorState(false); Advance(3500000);
  ASSERT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(board.state(),boards::State::Stopped);
  EXPECT_TRUE(board.channel(0).wires_swapped); // not double-applied across reset
}
TEST_F(BoardTest, WatchdogIsNotCommandInactivity) {
  Advance(3500000);
  client.SendSetCommand(0,CMD_STATE,uint8_t(3));
  client.SendSetCommand(0,CMD_MOTOR_VOLTAGE,int16_t(512)); Advance(100000);
  Advance(5000000); EXPECT_EQ(board.Voltage(0),1); // firmware keeps last command
  board.StallUntil(clock->now + 5000000); Advance(4100000);
  EXPECT_EQ(board.state(),boards::State::PreInit); EXPECT_EQ(board.Voltage(0),0);
}
TEST_F(BoardTest, DisconnectDoesNotLeaveReadyState) {
  Advance(3500000);
  link->open[1] = false;
  client.Tick(); EXPECT_FALSE(client.IsConnected()); EXPECT_FALSE(client.IsReady());
  EXPECT_FALSE(client.GetError().empty());
  link->open[1] = true; ASSERT_TRUE(client.Connect()); Advance(3500000);
  EXPECT_TRUE(client.IsReady()) << client.GetError();
}
TEST_F(BoardTest, ClientTimeoutIsExplicit) {
  Advance(3500000);
  // Stop emulator execution and drain the delivered bytes, then expire deadline.
  for (int i = 0; i < 100; ++i) { clock->now += 1000; client.Tick(); }
  clock->now += 11000000; client.Tick();
  EXPECT_FALSE(client.IsConnected()); EXPECT_EQ(client.GetError(),"board communication timeout");
}
TEST_F(BoardTest, PingsAndTemperatureDoNotRefreshEncoderFeedback) {
  Advance(3500000);
  ASSERT_TRUE(client.IsReady());
  // Drain any partial frame, then inject complete deterministic telemetry.
  link->baud = 0; link->latency_us = 0; link->max_chunk = 64;
  Advance(10000);
  for (int i = 0; i < 100; ++i) { clock->now += 100; client.Tick(); }
  link->incoming[0].clear();
  const auto inject = [&](const boards::Bytes& bytes) {
    for (auto byte : bytes) link->incoming[0].push_back({clock->now,byte});
    client.Tick();
  };
  inject({0x81,0,0,0,0});
  EXPECT_TRUE(client.HasFreshEncoderFeedback(100000));
  clock->now += 100000;
  inject({0x84,30,0,31,0,MESSAGE_TYPE_PING});
  EXPECT_TRUE(client.HasFreshEncoderFeedback(100000));
  ++clock->now;
  EXPECT_FALSE(client.HasFreshEncoderFeedback(100000));
  EXPECT_TRUE(client.IsReady()); // liveness/readiness alone is insufficient
  EXPECT_EQ(m0.GetTemperature(),30);
  inject({0x81,1,0,2,0});
  EXPECT_TRUE(client.HasFreshEncoderFeedback(100000));
  client.ResetBoard();
  EXPECT_FALSE(client.HasFreshEncoderFeedback(100000));
}

struct PoseHealthTest : ::testing::Test {
  std::shared_ptr<boards::ManualClock> clock = std::make_shared<boards::ManualClock>();
  std::map<std::string,std::shared_ptr<boards::MemoryLink>> links;
  std::map<std::string,std::unique_ptr<boards::EmulatedBoard>> emulators;
  std::unique_ptr<RobotSystem> robot;
  void SetUp() override {
    const auto config = YAML::LoadFile(std::string(getenv("TEST_SRCDIR")) +
        "/_main/src/odroid_controller/controller/config/robot_config.yaml");
    robot = std::make_unique<RobotSystem>(config,[&](const auto& name,const auto&) {
      auto link = std::make_shared<boards::MemoryLink>();
      links[name] = link;
      emulators[name] = std::make_unique<boards::EmulatedBoard>(
          std::make_unique<boards::MemoryTransport>(link,1,clock),clock,link);
      return std::make_unique<BLDCDriverBoard>(std::make_unique<boards::MemoryTransport>(link,0,clock),clock);
    });
    robot->Connect(); Advance(3500000);
    ASSERT_TRUE(robot->Ready());
    robot->HoldRotorPose(1,0.01,-1,28); Advance(50000);
    ASSERT_TRUE(robot->GetPoseFault().empty());
    for (const auto& [name,board] : emulators) ASSERT_EQ(board->state(),boards::State::Running);
  }
  void Advance(boards::Time duration) {
    const auto end = clock->now + duration;
    while (clock->now < end) {
      robot->Pump();
      for (auto& [name,board] : emulators) board->Pump();
      robot->Pump();
      if (clock->now % 1000 == 0) robot->Update(0.001f);
      clock->now += 500;
    }
  }
};
TEST_F(PoseHealthTest, StaleBoardStopsHealthyPeersAndLatchesUntilRestart) {
  emulators.at("controller_BZ")->StallUntil(clock->now + 200000);
  Advance(150000);
  EXPECT_NE(robot->GetPoseFault().find("controller_BZ: encoder feedback"),std::string::npos);
  for (const auto& [name,board] : emulators)
    if (name != "controller_BZ") { EXPECT_EQ(board->state(),boards::State::Stopped); }
  // A stalled MCU cannot consume STOP until it resumes (or its own WDT fires).
  Advance(100000);
  for (const auto& [name,board] : emulators) {
    EXPECT_EQ(board->state(),boards::State::Stopped);
    EXPECT_EQ(board->Voltage(0),0); EXPECT_EQ(board->Voltage(1),0);
  }
  EXPECT_TRUE(robot->Ready());
  EXPECT_THROW(robot->HoldRotorPose(1,0.01,-1,28),std::runtime_error);
  EXPECT_THROW(robot->HoldCurrentPose(),std::runtime_error);
}
TEST_F(PoseHealthTest, DisconnectStillStopsOtherBoards) {
  links.at("controller_BZ")->open[1] = false;
  EXPECT_NO_THROW(Advance(50000));
  EXPECT_FALSE(robot->GetPoseFault().empty());
  for (const auto& [name,board] : emulators)
    if (name != "controller_BZ") { EXPECT_EQ(board->state(),boards::State::Stopped); }
}
TEST_F(BoardTest, InvalidHeaderFailsClientWithoutCallbackReplay) {
  link->baud = 0; link->latency_us = 0; link->max_chunk = 64;
  Advance(3500000);
  link->incoming[0].clear();
  link->incoming[0].push_back({clock->now,0x30});
  client.Tick();
  EXPECT_FALSE(client.IsConnected());
  EXPECT_EQ(client.GetError(),"invalid board frame header");
}
TEST_F(BoardTest, OverTemperatureTriggersReset) {
  Advance(3500000);
  board.SetSensors(0,4090,60); board.Pump();
  Advance(20000);
  EXPECT_EQ(board.state(),boards::State::PreInit);
  EXPECT_EQ(board.Voltage(0),0);
  EXPECT_EQ(board.thermal_resets(),1u);
}
TEST_F(BoardTest, TemperatureTelemetryAndResetPreservePhysicalSamples) {
  temperature0 = 31; temperature1 = 42;
  Advance(3500000);
  ASSERT_TRUE(client.IsReady());
  EXPECT_EQ(m0.GetTemperature(),31); EXPECT_EQ(m1.GetTemperature(),42);
  temperature0 = 33; temperature1 = 40;
  Advance(50000);
  EXPECT_EQ(m0.GetTemperature(),33); EXPECT_EQ(m1.GetTemperature(),40);
  board.Reset();
  EXPECT_EQ(board.channel(0).temperature,33);
  EXPECT_EQ(board.channel(1).temperature,40);
  EXPECT_EQ(board.channel(0).encoder,4090);
  client.ResetBoard(); Advance(3500000);
  ASSERT_TRUE(client.IsReady());
  EXPECT_EQ(m0.GetTemperature(),33); EXPECT_EQ(m1.GetTemperature(),40);
}
TEST_F(BoardTest, WatchdogExpiresAtDeadlineAndClearsBothOutputs) {
  Advance(3500000);
  client.SendSetCommand(0,CMD_STATE,uint8_t(3));
  client.SendSetCommand(0,CMD_MOTOR_VOLTAGE,int16_t(512));
  client.SendSetCommand(1,CMD_MOTOR_VOLTAGE,int16_t(-512));
  Advance(50000);
  board.Pump();
  const auto fed = clock->now;
  board.StallUntil(fed + 5000000);
  clock->now = fed + boards::EmulatedBoard::kWatchdogTimeout - 1;
  board.Pump();
  EXPECT_EQ(board.Voltage(0),1); EXPECT_EQ(board.Voltage(1),-1);
  EXPECT_EQ(board.watchdog_resets(),0u);
  ++clock->now; board.Pump();
  EXPECT_EQ(board.watchdog_resets(),1u);
  EXPECT_EQ(board.thermal_resets(),0u);
  EXPECT_EQ(board.state(),boards::State::PreInit);
  EXPECT_EQ(board.Voltage(0),0); EXPECT_EQ(board.Voltage(1),0);
  EXPECT_EQ(board.channel(0).availability,0);
  EXPECT_EQ(board.channel(0).electrical_zero,0);
  client.ResetBoard(); Advance(3500000);
  EXPECT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(board.watchdog_resets(),1u); // ordinary 2 s sync does not expire WDT
}
TEST_F(BoardTest, WatchdogCannotBeFedAfterAnUnobservedExpiry) {
  Advance(3500000); board.Pump();
  board.StallUntil(clock->now + 4100000);
  clock->now += 4200000; // first Pump happens after both expiry and stall end
  board.Pump();
  EXPECT_EQ(board.watchdog_resets(),1u);
  EXPECT_EQ(board.state(),boards::State::PreInit);
  clock->now += boards::EmulatedBoard::kWatchdogTimeout;
  board.Pump(); // missing execution also expires without explicit fault injection
  EXPECT_EQ(board.watchdog_resets(),2u);
}
TEST_F(BoardTest, ShortStallResumesAndCriticalLoopFeedsWatchdog) {
  Advance(3500000); board.Pump();
  board.StallUntil(clock->now + 3000000);
  clock->now += 3000000; board.Pump();
  // Another 3 s is beyond the original deadline, but the resumed critical
  // iteration fed the watchdog. No incoming motor command is required.
  clock->now += 3000000; board.Pump();
  EXPECT_EQ(board.watchdog_resets(),0u);
  EXPECT_EQ(board.state(),boards::State::Stopped);
}
TEST_F(BoardTest, ThermalWatchdogBlocksCommandsAndRetainsHeat) {
  Advance(3500000);
  board.SetSensors(1,1234,60); board.Pump();
  const auto hot = clock->now;
  const auto commands = board.commands();
  client.SendSetCommand(0,CMD_STATE,uint8_t(3)); client.Tick();
  clock->now = hot + boards::EmulatedBoard::kThermalWatchdogTimeout - 1;
  board.Pump();
  EXPECT_EQ(board.commands(),commands);
  EXPECT_EQ(board.watchdog_resets(),0u);
  ++clock->now; board.Pump();
  EXPECT_EQ(board.watchdog_resets(),1u); EXPECT_EQ(board.thermal_resets(),1u);
  EXPECT_EQ(board.channel(1).temperature,60);
  EXPECT_EQ(board.Voltage(0),0);
  board.Pump(); // still hot after reboot: cannot erase the thermal fault
  clock->now += boards::EmulatedBoard::kThermalWatchdogTimeout; board.Pump();
  EXPECT_EQ(board.thermal_resets(),2u);
}
TEST(MemoryTransport, CapacityLatencyAndPartialProgress) {
  auto clock = std::make_shared<boards::ManualClock>();
  auto link = std::make_shared<boards::MemoryLink>();
  link->capacity = 2; link->latency_us = 200;
  boards::MemoryTransport a(link,0,clock), b(link,1,clock);
  a.Open(); b.Open();
  const uint8_t bytes[] = {1,2,3}; uint8_t read[3]{};
  EXPECT_EQ(a.Write(bytes,3),2);
  EXPECT_EQ(a.Write(bytes+2,1),0);
  EXPECT_EQ(b.Read(read,3),0);
  clock->now = 287;
  EXPECT_EQ(b.Read(read,3),1); EXPECT_EQ(read[0],1);
  clock->now = 374;
  EXPECT_EQ(b.Read(read,3),1); EXPECT_EQ(read[0],2);
  EXPECT_EQ(a.Pending(),0u);
  EXPECT_EQ(a.Write(bytes+2,1),1);
}
TEST(Readiness, InitAcknowledgementRequiresEncoderPacket) {
  auto clock = std::make_shared<boards::ManualClock>();
  auto link = std::make_shared<boards::MemoryLink>();
  link->baud = 0; link->latency_us = 0;
  boards::MemoryTransport peer(link,1,clock); peer.Open();
  BLDCDriverBoard client(std::make_unique<boards::MemoryTransport>(link,0,clock),clock);
  Motor motor(&client,0); motor.UpdateConfig(MotorConfig()); client.SetMotor(0,&motor);
  ASSERT_TRUE(client.Connect()); clock->now = 1000000; client.Tick();
  uint8_t sync[] = {0x0f}; peer.Write(sync,sizeof(sync)); client.Tick();
  uint8_t init[] = {0xc2,4,'I','N','I','C'}; peer.Write(init,sizeof(init)); client.Tick();
  EXPECT_FALSE(client.IsReady());
  uint8_t encoder[] = {0x81,0xd2,0x04,0,0}; peer.Write(encoder,sizeof(encoder)); client.Tick();
  EXPECT_TRUE(client.IsReady()); EXPECT_EQ(motor.GetRawAngle(),1234);
}
TEST(SerialTransport, PtyRunsSameCodecAndBoard) {
  auto clock = std::make_shared<boards::ManualClock>();
  int fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(fd,0); ASSERT_EQ(grantpt(fd),0); ASSERT_EQ(unlockpt(fd),0);
  std::string slave = ptsname(fd);
  boards::EmulatedBoard emulator(std::make_unique<boards::SerialTransport>(fd,clock),clock);
  BLDCDriverBoard client(std::make_unique<boards::SerialTransport>(slave,clock),clock);
  Motor m0(&client,0), m1(&client,1);
  m0.UpdateConfig(MotorConfig()); m1.UpdateConfig(MotorConfig());
  client.SetMotor(0,&m0); client.SetMotor(1,&m1);
  emulator.SetSensors(0,4090,31); emulator.SetSensors(1,1234,42);
  ASSERT_TRUE(client.Connect());
  for (int i = 0; i < 8000; ++i) {
    clock->now += 500; client.Tick(); emulator.Pump(); client.Tick();
  }
  EXPECT_TRUE(client.IsReady()) << client.GetError();
  EXPECT_EQ(m0.GetTemperature(),31); EXPECT_EQ(m1.GetTemperature(),42);
  int replies = 0;
  client.Query(0,CMD_STATE,[&](const auto& b) { EXPECT_EQ(b[0],2); ++replies; });
  for (int i = 0; i < 100; ++i) {
    clock->now += 500; client.Tick(); emulator.Pump(); client.Tick();
  }
  EXPECT_EQ(replies,1);
  emulator.SetSensors(0,4090,35); emulator.SetSensors(1,1234,44);
  for (int i = 0; i < 100; ++i) {
    clock->now += 500; client.Tick(); emulator.Pump(); client.Tick();
  }
  EXPECT_EQ(m0.GetTemperature(),35); EXPECT_EQ(m1.GetTemperature(),44);
  emulator.StallUntil(clock->now + 5000000);
  clock->now += boards::EmulatedBoard::kWatchdogTimeout;
  emulator.Pump();
  EXPECT_EQ(emulator.watchdog_resets(),1u);
  EXPECT_EQ(emulator.channel(1).temperature,44);
  client.ResetBoard();
  EXPECT_FALSE(client.IsConnected()); // unsupported reset is explicit for PTYs
}
}  // namespace
