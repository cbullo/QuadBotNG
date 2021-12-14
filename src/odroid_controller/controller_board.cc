#include "controller_board.h"

#include <asm/ioctls.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <iostream>

#include "motor.h"

ControllerBoard::ControllerBoard() {}

void ControllerBoard::UpdateConfig(const YAML::Node &config) {
  usb_address_ = config["controller_address"].as<std::string>();
}

bool ControllerBoard::Connect() {
  serial_ = open(usb_address_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_ == -1) {
    std::cout << "Failed to open port" << std::endl;
    return false;
  }

  if (!isatty(serial_)) {
    Disconnect();
    return false;
  }

  struct termios config;
  if (tcgetattr(serial_, &config) < 0) {
    Disconnect();
    return false;
  }

  config.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

  config.c_oflag &= ~OPOST;
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 0;

  if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
    Disconnect();
    return false;
  }

  if (tcsetattr(serial_, TCSANOW, &config) < 0) {
    Disconnect();
    return false;
  }

  struct serial_struct ser_info;
  ioctl(serial_, TIOCGSERIAL, &ser_info);
  ser_info.flags |= ASYNC_LOW_LATENCY;
  ioctl(serial_, TIOCSSERIAL, &ser_info);

  StartProcessThread();
  return true;
}

void ControllerBoard::Disconnect() {
  if (serial_ != -1) {
    close(serial_);
  }
  std::cout << "Disconnected" << std::endl;
}

bool ControllerBoard::IsConnected() const { return serial_ != -1; }

void ControllerBoard::StartProcessThread() {
  //communication_thread =
  //    std::make_unique<std::thread>(&ControllerBoard::ProcessLoop, this);
}

void ControllerBoard::ProcessLoop() {
  //while (IsConnected()) {

  //}
}

void ControllerBoard::HardwareReset() const {
  int RTS_flag = TIOCM_RTS;
  ioctl(serial_, TIOCMBIS, &RTS_flag);  // Set RTS pin
  usleep(10000);
  ioctl(serial_, TIOCMBIC, &RTS_flag);  // Clear RTS pin
}

void ControllerBoard::SendCommand(const std::string& command) const {
  write(serial_, command.c_str(), command.size());
  const auto eol = "\n";
  write(serial_, &eol, 1);
}
