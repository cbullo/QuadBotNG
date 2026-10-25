#include "bldc_driver_board.h"
#include "motor.h"
#include <iostream>
#include <stdexcept>

BLDCDriverBoard::BLDCDriverBoard() : clock_(std::make_shared<boards::SteadyClock>()) {}
BLDCDriverBoard::BLDCDriverBoard(std::unique_ptr<boards::Transport> transport,
                               std::shared_ptr<boards::Clock> clock)
    : transport_(std::move(transport)), clock_(std::move(clock)) {}
BLDCDriverBoard::~BLDCDriverBoard() { if (transport_) transport_->Close(); }
void BLDCDriverBoard::UpdateConfig(const YAML::Node& config) {
  address_ = config["controller_address"].as<std::string>();
}
void BLDCDriverBoard::Clear() {
  incoming_.clear(); outgoing_.clear(); replies_.clear();
  initialized_ = false;
  sensor_received_.fill(false);
  encoder_received_at_.fill(0);
  temperature_received_.fill(false);
  temperature_received_at_.fill(0);
  voltage_command_.fill(0);
  state_command_ = -1;
  for (auto* motor : motors_) if (motor) motor->ResetFeedback();
}
bool BLDCDriverBoard::HasFreshEncoderFeedback(boards::Time maximum_age) const {
  if (!IsReady()) return false;
  for (int i = 0; i < 2; ++i)
    if (motors_[i] && clock_->Now() - encoder_received_at_[i] > maximum_age) return false;
  return true;
}
bool BLDCDriverBoard::Connect() {
  if (!transport_) transport_ = std::make_unique<boards::SerialTransport>(address_, clock_);
  if (IsConnected()) Disconnect();
  Clear();
  error_.clear(); error_state_ = false;
  if (!transport_->Open()) { error_ = "cannot open board transport"; return false; }
  phase_ = Phase::Boot;
  deadline_ = clock_->Now() + 1000000;
  return true;
}
void BLDCDriverBoard::Disconnect() {
  if (transport_) transport_->Close();
  phase_ = Phase::Disconnected;
  Clear();
}
void BLDCDriverBoard::Fail(const std::string& message) {
  error_ = message;
  error_state_ = true;
  // Best effort physical reset. Unsupported PTY reset is not treated as success.
  if (transport_ && transport_->Connected()) transport_->Reset();
  Disconnect();
}
void BLDCDriverBoard::ResetBoard() {
  if (!IsConnected() || !transport_->Reset()) {
    Fail("transport does not support board reset"); return;
  }
  Clear(); phase_ = Phase::Boot;
  deadline_ = clock_->Now() + 1000000;
}
void BLDCDriverBoard::SetErrorState(bool enabled) {
  if (enabled && !error_state_) {
    ResetBoard();
    error_state_ = true;  // latch: caller must explicitly release before resync
  } else if (!enabled) error_state_ = false;
}
void BLDCDriverBoard::Enqueue(boards::Bytes bytes, Reply callback) {
  boards::ValidateCommand(bytes);
  if (outgoing_.size() >= 512) throw std::runtime_error("board command queue full");
  outgoing_.push_back({std::move(bytes), 0, std::move(callback)});
}
void BLDCDriverBoard::SendSync() {
  outgoing_.push_front({boards::Bytes{CMD_SYNC}, 0, {}});
  phase_ = Phase::Syncing;
  deadline_ = clock_->Now() + 4000000;
}
void BLDCDriverBoard::Query(uint8_t motor, uint8_t command, Reply cb, int subcommand) {
  if (motor > 1 || command > MAIN_COMMAND_MASK) throw std::invalid_argument("invalid motor/command");
  boards::Bytes bytes{uint8_t(command | GET_CMD_BIT | (motor ? MOTOR_INDEX_BIT : 0))};
  if (subcommand >= 0) bytes.push_back(subcommand);
  Enqueue(std::move(bytes), std::move(cb));
}
void BLDCDriverBoard::SendGetCommand(uint8_t m, uint8_t c) { Query(m, c, {}); }
void BLDCDriverBoard::SendGetCommand(uint8_t m, uint8_t c, uint8_t s) { Query(m, c, {}, s); }
void BLDCDriverBoard::Setup() {
  auto waiting = std::move(outgoing_);
  outgoing_.clear();
  for (uint8_t i = 0; i < 2; ++i) {
    auto* m = motors_[i];
    SendSetCommand(i, CMD_AVAILABILITY, uint8_t(m ? 3 : 0));
    if (!m) continue;
    SendSetCommandAndSub(i, CMD_SENSOR_LINEARIZATION, SCMD_OFFSET,
                        m->sensor_linearization_offset_, m->swap_wires_);
    if (m->sensor_linearization_coeffs_.size() != 16)
      throw std::runtime_error("expected 16 encoder calibration coefficients");
    for (uint8_t j = 0; j < 16; ++j)
      SendSetCommandAndSub(i, CMD_SENSOR_LINEARIZATION, SCMD_FACTOR, j,
                          m->sensor_linearization_coeffs_[j]);
    SendSetCommandAndSub(i, CMD_SENSOR, SCMD_SENS_ELEC_OFFSET, int16_t(m->electric_zero_angle_));
  }
  SendSetCommand(0, CMD_STATE, uint8_t(1));
  for (auto& command : waiting) outgoing_.push_back(std::move(command));
}
void BLDCDriverBoard::Receive(const boards::Bytes& frame) {
  const auto h = frame[0];
  if (phase_ == Phase::Syncing) {
    if (h != MESSAGE_SUBTYPE_SYNC) throw std::runtime_error("expected board sync reply");
    phase_ = Phase::Ready;
    Setup();
  } else if ((h & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_DATA_STREAM) {
    for (int i = 0; i < 2; ++i) if (auto* m = motors_[i]) {
      auto value = boards::U16(frame.data() + 1 + 2*i);
      if (h & DATA_STREAM_ANGLE_BIT) {
        m->SetCurrentRawAngle(value);
        sensor_received_[i] = true;
        encoder_received_at_[i] = clock_->Now();
      }
      else {
        m->SetTemperature(value);
        temperature_received_[i] = true;
        temperature_received_at_[i] = clock_->Now();
      }
    }
  } else if ((h & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_GET_REPLY) {
    if (replies_.empty() || replies_.front().command != GetCommand(h))
      throw std::runtime_error("unexpected GET reply");
    auto reply = std::move(replies_.front()); replies_.pop_front();
    if (frame.back() != 0) throw std::runtime_error("unexpected GET sequence");
    if (reply.callback) reply.callback(boards::Bytes(frame.begin()+1, frame.end()-1));
  } else if ((h & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_STRING) {
    if (std::string(frame.begin()+2, frame.end()) == "INIC") initialized_ = true;
    if ((h & MESSAGE_SUBTYPE_MASK) == STRING_MESSAGE_TYPE_ERROR)
      throw std::runtime_error(std::string(frame.begin()+2, frame.end()));
  } else if (h != MESSAGE_TYPE_PING) {
    throw std::runtime_error("unexpected board frame");
  }
  deadline_ = clock_->Now() + 10000000;
}
void BLDCDriverBoard::Pump() {
  if (!IsConnected()) return;
  // Also service a pending asynchronous reset while error is latched.
  uint8_t buffer[256];
  try {
    if (phase_ == Phase::Boot || error_state_) {
      int n = transport_->Read(buffer, sizeof(buffer));  // discard boot output
      if (n < 0) { Fail("board transport disconnected"); return; }
      if (error_state_ || clock_->Now() < deadline_) return;
      SendSync();
    }
    for (int budget = 0; budget < 16; ++budget) {
      int n = transport_->Read(buffer, sizeof(buffer));
      if (n < 0) { Fail("board transport disconnected"); return; }
      if (!n) break;
      incoming_.insert(incoming_.end(), buffer, buffer+n);
      while (!incoming_.empty()) {
        auto size = boards::MessageSize(incoming_);
        if (!size || incoming_.size() < size) break;
        boards::Bytes frame(incoming_.begin(), incoming_.begin()+size);
        incoming_.erase(incoming_.begin(), incoming_.begin()+size);
        Receive(frame);
      }
    }
    if (clock_->Now() > deadline_) { Fail("board communication timeout"); return; }
    if (!replies_.empty() && clock_->Now() > replies_.front().deadline) {
      Fail("GET reply timeout"); return;
    }
    for (int budget = 0; budget < 64 && !outgoing_.empty(); ++budget) {
      auto& command = outgoing_.front();
      if (phase_ != Phase::Ready && command.bytes[0] != CMD_SYNC) break;
      int n = transport_->Write(command.bytes.data()+command.offset,
                               command.bytes.size()-command.offset);
      if (n < 0) { Fail("board write failed"); return; }
      if (!n) break;
      command.offset += n;
      if (command.offset == command.bytes.size()) {
        if (!IsGetMessage(command.bytes[0])) {
          if (GetCommand(command.bytes[0]) == CMD_MOTOR_VOLTAGE)
            voltage_command_[(command.bytes[0] & MOTOR_INDEX_BIT) ? 1 : 0] = boards::I16(command.bytes.data()+1);
          if (GetCommand(command.bytes[0]) == CMD_STATE) state_command_ = command.bytes[1];
        }
        if (IsGetMessage(command.bytes[0]))
          replies_.push_back({GetCommand(command.bytes[0]), std::move(command.callback),
                              clock_->Now()+1000000});
        outgoing_.pop_front();
      }
    }
  } catch (const std::exception& e) { Fail(e.what()); }
}
void BLDCDriverBoard::Tick() {
  Pump();
  for (auto* motor : motors_) if (motor) motor->Update();
}
