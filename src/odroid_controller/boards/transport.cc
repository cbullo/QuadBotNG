#include "transport.h"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace boards {
SerialTransport::SerialTransport(std::string path, std::shared_ptr<Clock> clock)
    : path_(std::move(path)), clock_(std::move(clock)) {}
SerialTransport::SerialTransport(int fd, std::shared_ptr<Clock> clock)
    : clock_(std::move(clock)), fd_(fd), pty_master_(true) {
  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL) | O_NONBLOCK);
}
bool SerialTransport::Open() {
  if (fd_ >= 0) return true;
  fd_ = open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) return false;
  termios config{};
  if (tcgetattr(fd_, &config) < 0) { Close(); return false; }
  cfmakeraw(&config);
  config.c_cflag |= CLOCAL | CREAD;
  config.c_cflag &= ~(CSTOPB | CRTSCTS);
  config.c_cc[VMIN] = 0;
  config.c_cc[VTIME] = 0;
  if (cfsetispeed(&config, B115200) || cfsetospeed(&config, B115200) ||
      tcsetattr(fd_, TCSANOW, &config)) { Close(); return false; }
  serial_struct info{};
  if (ioctl(fd_, TIOCGSERIAL, &info) == 0) {
    info.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &info);  // optional USB driver optimization
  }
  return true;
}
void SerialTransport::Close() {
  if (fd_ >= 0) close(fd_);
  fd_ = -1;
  release_reset_ = 0;
}
bool SerialTransport::ServiceReset() {
  if (!release_reset_) return true;
  if (clock_->Now() < release_reset_) return false;
  int flag = TIOCM_RTS;
  if (ioctl(fd_, TIOCMBIC, &flag) < 0) { Close(); return false; }
  release_reset_ = 0;
  tcflush(fd_, TCIOFLUSH);
  return true;
}
int SerialTransport::Read(uint8_t* b, size_t n) {
  if (fd_ < 0) return -1;
  if (!ServiceReset()) return fd_ >= 0 ? 0 : -1;
  const auto r = read(fd_, b, n);
  if (r < 0 && errno == EIO && pty_master_) return -1;
  if (r < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
  if (r < 0) { Close(); return -1; }
  return static_cast<int>(r);
}
int SerialTransport::Write(const uint8_t* b, size_t n) {
  if (fd_ < 0) return -1;
  if (!ServiceReset()) return fd_ >= 0 ? 0 : -1;
  const auto r = write(fd_, b, n);
  if (r < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
  if (r < 0) { Close(); return -1; }
  return static_cast<int>(r);
}
size_t SerialTransport::Pending() const {
  int count = 0;
  if (fd_ >= 0 && ioctl(fd_, TIOCOUTQ, &count) == 0) return count;
  return 0;
}
bool SerialTransport::Reset() {
  int flag = TIOCM_RTS;
  if (fd_ < 0 || ioctl(fd_, TIOCMBIS, &flag) < 0) return false;
  release_reset_ = clock_->Now() + 10000;
  return true;
}
void MemoryLink::Reset() {
  incoming[0].clear(); incoming[1].clear();
  next_tx[0] = next_tx[1] = 0;
  ++generation;
}
int MemoryTransport::Read(uint8_t* bytes, size_t count) {
  if (!Connected() || !link_->open[1-side_]) return -1;
  auto& q = link_->incoming[side_];
  size_t n = 0;
  while (n < std::min(count, link_->max_chunk) && !q.empty() && q.front().due <= clock_->Now()) {
    bytes[n++] = q.front().value; q.pop_front();
  }
  return n;
}
int MemoryTransport::Write(const uint8_t* bytes, size_t count) {
  if (!Connected() || !link_->open[1-side_]) return -1;
  auto& q = link_->incoming[1-side_];
  size_t n = std::min({count, link_->max_chunk, link_->capacity - q.size()});
  auto& next = link_->next_tx[side_];
  for (size_t i = 0; i < n; ++i) {
    next = std::max(next, clock_->Now()) + (link_->baud ? (10000000 + link_->baud-1)/link_->baud : 0);
    q.push_back({next + link_->latency_us, bytes[i]});
  }
  return n;
}
size_t MemoryTransport::Pending() const {
  // Bytes in flight, not unread bytes already delivered at the other endpoint.
  const auto& q = link_->incoming[1-side_];
  return std::count_if(q.begin(), q.end(), [&](const auto& b) { return b.due > clock_->Now(); });
}
}  // namespace boards
