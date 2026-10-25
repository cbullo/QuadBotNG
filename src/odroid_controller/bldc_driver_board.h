#pragma once
#include <array>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include "boards/protocol.h"
#include "boards/transport.h"
#include "yaml-cpp/yaml.h"

class Motor;
// Common protocol client. All calls run on the controller/runner thread.
// Hardware and simulation differ only in the injected transport and clock.
class BLDCDriverBoard {
 public:
  using Reply = std::function<void(const boards::Bytes&)>;
  BLDCDriverBoard();
  BLDCDriverBoard(std::unique_ptr<boards::Transport>, std::shared_ptr<boards::Clock>);
  ~BLDCDriverBoard();
  void UpdateConfig(const YAML::Node&);
  void SetMotor(int index, Motor* motor) { motors_.at(index) = motor; }
  Motor* GetMotor(int index) { return motors_.at(index); }
  const Motor* GetMotor(int index) const { return motors_.at(index); }
  void SetName(const std::string& name) { name_ = name; }
  std::string GetName() const { return name_; }
  std::string GetUSBAddress() const { return address_; }
  bool Connect();
  void Disconnect();
  bool IsConnected() const { return transport_ && transport_->Connected(); }
  bool IsReady() const {
    if (phase_ != Phase::Ready || !initialized_ || error_state_) return false;
    for (int i = 0; i < 2; ++i) if (motors_[i] && !sensor_received_[i]) return false;
    return true;
  }
  const std::string& GetError() const { return error_; }
  bool HasFreshEncoderFeedback(boards::Time maximum_age) const;
  void Tick();
  void Pump();
  void ResetBoard();
  void HardwareReset() { ResetBoard(); }
  void SetErrorState(bool enabled);
  bool IsTransmissionBound() const { return transport_ && transport_->Pending() > 0; }
  bool AreCommandsScheduled() const { return !outgoing_.empty(); }
  void SendSync();
  void SendGetCommand(uint8_t motor, uint8_t command);
  void SendGetCommand(uint8_t motor, uint8_t command, uint8_t subcommand);
  void Query(uint8_t motor, uint8_t command, Reply callback, int subcommand = -1);

  template<class... Args> void SendSetCommand(uint8_t motor, uint8_t command, Args... args) {
    if (motor > 1 || command > MAIN_COMMAND_MASK) throw std::invalid_argument("invalid motor/command");
    boards::Bytes bytes{uint8_t((command & MAIN_COMMAND_MASK) | (motor ? MOTOR_INDEX_BIT : 0))};
    (boards::Append(bytes, args), ...);
    Enqueue(std::move(bytes));
  }
  template<class... Args> void SendSetCommandAndSub(uint8_t motor, uint8_t command,
                                                  uint8_t subcommand, Args... args) {
    SendSetCommand(motor, command, subcommand, args...);
  }
 private:
  enum class Phase { Disconnected, Boot, Syncing, Ready };
  struct Outgoing { boards::Bytes bytes; size_t offset = 0; Reply callback; };
  struct PendingReply { uint8_t command; Reply callback; boards::Time deadline; };
  void Enqueue(boards::Bytes, Reply callback = {});
  void Setup();
  void Receive(const boards::Bytes&);
  void Fail(const std::string&);
  void Clear();
  std::unique_ptr<boards::Transport> transport_;
  std::shared_ptr<boards::Clock> clock_;
  std::array<Motor*, 2> motors_{};
  std::array<bool, 2> sensor_received_{};
  std::array<boards::Time, 2> encoder_received_at_{};
  std::string name_, address_, error_;
  Phase phase_ = Phase::Disconnected;
  bool error_state_ = false;
  bool initialized_ = false;
  boards::Time deadline_ = 0;
  boards::Bytes incoming_;
  std::deque<Outgoing> outgoing_;
  std::deque<PendingReply> replies_;
};
