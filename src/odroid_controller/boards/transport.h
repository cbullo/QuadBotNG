#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>

namespace boards {
using Time = int64_t;  // monotonic microseconds
struct Clock {
  virtual ~Clock() = default;
  virtual Time Now() const = 0;
};
struct SteadyClock final : Clock {
  Time Now() const override {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
};
struct ManualClock final : Clock {
  Time now = 0;
  Time Now() const override { return now; }
};

// All operations are nonblocking and called by one owning runner thread.
// Read/Write: positive progress, 0 would-block, -1 disconnected/error.
struct Transport {
  virtual ~Transport() = default;
  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool Connected() const = 0;
  virtual int Read(uint8_t* bytes, size_t count) = 0;
  virtual int Write(const uint8_t* bytes, size_t count) = 0;
  virtual size_t Pending() const = 0;
  virtual bool Reset() = 0;
};

class SerialTransport final : public Transport {
 public:
  SerialTransport(std::string path, std::shared_ptr<Clock> clock);
  // Adopt an already-open PTY master for emulator integration.
  SerialTransport(int fd, std::shared_ptr<Clock> clock);
  ~SerialTransport() override { Close(); }
  bool Open() override;
  void Close() override;
  bool Connected() const override { return fd_ >= 0; }
  int Read(uint8_t*, size_t) override;
  int Write(const uint8_t*, size_t) override;
  size_t Pending() const override;
  bool Reset() override;
 private:
  bool ServiceReset();
  std::string path_;
  std::shared_ptr<Clock> clock_;
  int fd_ = -1;
  bool pty_master_ = false;
  Time release_reset_ = 0;
};

struct MemoryLink {
  struct Byte { Time due; uint8_t value; };
  std::deque<Byte> incoming[2];
  bool open[2] = {false, false};
  Time next_tx[2] = {0, 0};
  Time latency_us = 200;
  unsigned baud = 115200;  // 0 is an explicitly ideal test link
  size_t capacity = 256;
  size_t max_chunk = 64;
  uint64_t generation = 0;
  void Reset();
};
class MemoryTransport final : public Transport {
 public:
  MemoryTransport(std::shared_ptr<MemoryLink> link, int side, std::shared_ptr<Clock> clock)
      : link_(std::move(link)), side_(side), clock_(std::move(clock)) {}
  bool Open() override { link_->open[side_] = true; return true; }
  void Close() override { link_->open[side_] = false; link_->Reset(); }
  bool Connected() const override { return link_->open[side_]; }
  int Read(uint8_t*, size_t) override;
  int Write(const uint8_t*, size_t) override;
  size_t Pending() const override;
  bool Reset() override { link_->Reset(); return true; }
 private:
  std::shared_ptr<MemoryLink> link_;
  int side_;
  std::shared_ptr<Clock> clock_;
};
}  // namespace boards
