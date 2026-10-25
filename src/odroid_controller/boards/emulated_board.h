#pragma once
#include <array>
#include "protocol.h"
#include "transport.h"

namespace boards {
enum class State : uint8_t { PreInit, Init, Stopped, Running, Error };
struct Channel {
  uint8_t availability = 0;
  int16_t voltage = 0;
  int16_t voltage_limit = 28 * 512 - 1;  // BLDCMotor constructor default
  int16_t electrical_zero = 0;
  uint8_t linearization_offset = 0;
  std::array<uint8_t, 16> linearization{};
  bool wires_swapped = false;
  uint8_t motion_type = 0;
  uint8_t centered = 0;
  uint16_t encoder = 0;
  uint16_t temperature = 20;
};
class EmulatedBoard {
 public:
  static constexpr Time kWatchdogTimeout = 4000000;  // WDTO_4S
  static constexpr Time kThermalWatchdogTimeout = 15000;  // WDTO_15MS
  EmulatedBoard(std::unique_ptr<Transport> transport, std::shared_ptr<Clock> clock,
                std::shared_ptr<MemoryLink> link = {});
  void Pump();
  void Reset();
  void SetSensors(int channel, uint16_t encoder, uint16_t temperature = 20);
  double Voltage(int channel) const;
  State state() const { return state_; }
  const Channel& channel(int i) const { return channels_.at(i); }
  uint64_t commands() const { return commands_; }
  uint64_t watchdog_resets() const { return watchdog_resets_; }
  uint64_t thermal_resets() const { return thermal_resets_; }
  // Explicit execution-watchdog fault injection, distinct from loss of commands.
  void StallUntil(Time until) { stalled_until_ = until; }
 private:
  void Process(const Bytes&);
  void String(uint8_t level, const std::string& text);
  void Flush();
  void Reply(uint8_t command, uint16_t value, size_t width);
  void WatchdogReset(bool thermal);
  std::unique_ptr<Transport> transport_;
  std::shared_ptr<Clock> clock_;
  std::shared_ptr<MemoryLink> link_;
  uint64_t generation_ = 0, commands_ = 0;
  uint64_t watchdog_resets_ = 0, thermal_resets_ = 0;
  std::array<Channel, 2> channels_{};
  State state_ = State::PreInit;
  Bytes input_, output_;
  Time sync_due_ = -1, init_due_ = -1, next_telemetry_ = 0;
  Time watchdog_due_ = 0, stalled_until_ = 0, thermal_due_ = -1;
  Time next_ping_ = 0;
  bool synced_ = false, temperature_next_ = false;
};
}  // namespace boards
