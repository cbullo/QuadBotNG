#include "bldc_driver_board.h"

#include <asm/ioctls.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "motor.h"
#include "src/libs/communication/binary_stream.h"

BLDCDriverBoard::BLDCDriverBoard() {}

void BLDCDriverBoard::UpdateConfig(const YAML::Node &config) {
  usb_address_ = config["controller_address"].as<std::string>();
}

bool BLDCDriverBoard::Connect() {
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

  StartCommunicationThread();
  return true;
}

void BLDCDriverBoard::Disconnect() {
  if (serial_ != -1) {
    close(serial_);
  }
  std::cout << "Disconnected" << std::endl;
}

bool BLDCDriverBoard::IsConnected() const { return serial_ != -1; }

void BLDCDriverBoard::StartCommunicationThread() {
  communication_thread_ =
      std::make_unique<std::thread>(&BLDCDriverBoard::ProcessLoop, this);
}

void BLDCDriverBoard::ProcessSend() {
  if (pending_commands_.empty()) {
    return;
  }

  Command to_process;
  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    if (!pending_commands_.empty()) {
      to_process = pending_commands_.front();
      pending_commands_.pop();
      if (to_process.command_bytes[0] & GET_CMD_BIT) {
        sent_get_commands_.push(to_process);
      }
    }
  }

  SendCommand(to_process.command_bytes, to_process.command_length);
}

void BLDCDriverBoard::SendCommand(const uint8_t *bytes, int message_length) {
  if (message_length == 0) {
    assert(false);
    return;
  }
  while (message_length) {
    int bytes_sent = write(serial_, bytes, message_length);
    if (bytes_sent == -1) {
      if (errno == EAGAIN) {
        continue;
      } else {
        HandleError();
      }
    }
    message_length -= bytes_sent;
  }
}

void BLDCDriverBoard::DiscardReceive() {
  ssize_t bytes_read;
  while (true) {
    int value;
    int error = ioctl(serial_, FIONREAD, &value);
    if (error == -1) {
      HandleError();
    }
    if (value == 0) {
      break;
    }
    bytes_read = read(serial_, serial_read_buffer_, value);
    if (bytes_read == -1) {
      if (errno == EAGAIN) {
        break;
      } else {
        HandleError();
        break;
      }
    }
  }
}

uint8_t BLDCDriverBoard::CalculateReplySize(uint8_t header_byte) {
  return GetArgumentLength(header_byte);
}

uint8_t BLDCDriverBoard::CalculateDataStreamSize(uint8_t header_byte) {
  uint8_t data_fields = header_byte & MESSAGE_SUBTYPE_MASK;
  uint8_t data_size = 0;

  // TODO: handle this better - share data type with MCU
  if (data_fields & DATA_STREAM_ANGLE_BIT) {
    data_size += sizeof(float);
  } else if (data_fields & DATA_STREAM_VELOCITY_BIT) {
    data_size += sizeof(float);
  } else if (data_fields & DATA_STREAM_TEMPERATURE_BIT) {
    data_size += sizeof(float);
  }
  data_size *= 2;  // Always return values for both motors
  return data_size;
}

// Returns size of dynamic message data (excluding header and static data)
uint8_t BLDCDriverBoard::ProcessStaticData(const uint8_t *bytes) {
  uint8_t message_type = bytes[0] & MESSAGE_TYPE_MASK;
  switch (message_type) {
    case MESSAGE_TYPE_STRING:
      return bytes[1];
    case MESSAGE_TYPE_GET_REPLY:
      return CalculateReplySize(message_type);
    case MESSAGE_TYPE_DATA_STREAM:
      return CalculateDataStreamSize(message_type);
    default:
      return 0;
  }
}

void BLDCDriverBoard::ProcessReceive() {
  size_t nbytes;
  ssize_t bytes_read;
  int fd = serial_;
  while (true) {
    int value;
    int error = ioctl(fd, FIONREAD, &value);
    if (error == -1) {
      HandleError();
      return;
    }
    if (value == 0) {
      break;
    }
    bytes_read = read(fd, serial_read_buffer_ + read_offset_,
                      std::min(value, static_cast<int>(expected_reply_bytes_)));
    if (bytes_read == -1) {
      if (errno == EAGAIN) {
        break;
      } else {
        HandleError();
        return;
      }
    } else {
      read_offset_ += bytes_read;

      if (read_offset_ == expected_msg_bytes_) {
        switch (msg_part_) {
          case ExpectedMessagePart::kHeaderPart:
            expected_msg_bytes_ =
                ProcessHeader(serial_read_buffer_[0]) + read_offset_;
            msg_part_ = ExpectedMessagePart::kStaticPart;
            break;
          case ExpectedMessagePart::kStaticPart:
            expected_msg_bytes_ =
                ProcessStaticData(serial_read_buffer_) + read_offset_;
            msg_part_ = ExpectedMessagePart::kDynamicPart;
            break;
          case ExpectedMessagePart::kDynamicPart:
            ProcessMessage(serial_read_buffer_, read_offset_);
            expected_reply_bytes_ = MESSAGE_HEADER_SIZE;
            read_offset_ = 0;
        }
      }
    }
  }
}

// Returns size of static message data (excluding header)
uint8_t BLDCDriverBoard::ProcessHeader(uint8_t header_byte) {
  switch (header_byte & MESSAGE_TYPE_MASK) {
    case MESSAGE_TYPE_SYNC:
      return 0;
    case MESSAGE_TYPE_PING:
      return 0;
    case MESSAGE_TYPE_GET_REPLY: {
      // Make sure we're waiting for this message
      auto cmd = sent_get_commands_.front();
      if ((cmd.command_bytes[0] & MESSAGE_TYPE_MASK) !=
          (header_byte & MESSAGE_TYPE_MASK)) {
        HandleError();
      }
      return 0;
    }
    case MESSAGE_TYPE_DATA_STREAM:
      return 0;
    case MESSAGE_TYPE_STRING:
      return 1;
  }
}

void BLDCDriverBoard::ProcessMessage(const uint8_t *message, int message_size) {
  // Make sure we're getting data that we're expecting
  switch (sync_state_) {
    case SyncState::kNotSynced:
    case SyncState::kSyncingPause:
      // We shouldn't bother calling this if not synced.
      assert(false);
      break;
    case SyncState::kSyncing:
      if (message_size != 1 ||
          message[0] != (MESSAGE_TYPE_SYNC | MESSAGE_SUBTYPE_SYNC)) {
        HandleError();
        return;
      }
      SetState(SyncState::kSynced);
      break;
    case SyncState::kSynced:
      if (message[0] & MESSAGE_TYPE_MASK == MESSAGE_TYPE_PING) {
        return;
      }
      break;
  }

  Command read_command;
  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    read_command = sent_get_commands_.front();
  }
  read_command.callback->ReadArgs(message, message_size);
  {
    std::lock_guard<std::mutex> guard(reply_mutex_);
    replied_commands_queue_.push(read_command);
  }
}

void BLDCDriverBoard::ProcessLoop() {
  while (IsConnected()) {
    switch (sync_state_) {
      case SyncState::kNotSynced:
        DiscardReceive();
        ProcessSend();
        break;
      case SyncState::kSyncingPause:
        DiscardReceive();
        if (std::chrono::system_clock::now() >= communication_deadline_) {
          SetState(SyncState::kSyncing);
          BumpTimeout(3000);
        }
        break;
      case SyncState::kSyncing:
        ProcessReceive();
        break;
      case SyncState::kSynced:
        ProcessReceive();
        ProcessSend();
        break;
    }
  }
  CheckTimeout();
}

void BLDCDriverBoard::HardwareReset() const {
  int RTS_flag = TIOCM_RTS;
  ioctl(serial_, TIOCMBIS, &RTS_flag);  // Set RTS pin
  usleep(10000);
  ioctl(serial_, TIOCMBIC, &RTS_flag);  // Clear RTS pin
}

void BLDCDriverBoard::SendSync() {
  SendCommand(CMD_SYNC);
  BumpTimeout(1000);
  SetState(SyncState::kSyncingPause);
}

void BLDCDriverBoard::CheckTimeout() {
  if (std::chrono::system_clock::now() >= communication_deadline_) {
    HandleError();
  }
}

void BLDCDriverBoard::BumpTimeout(int ms) {
  communication_deadline_ =
      std::chrono::system_clock::now() +
      std::chrono::milliseconds{ms};
}

void BLDCDriverBoard::SetState(SyncState new_state) { sync_state_ = new_state; }

void BLDCDriverBoard::SendCommand(COMMAND_TYPE command) {
  Command cmd;
  cmd.command_bytes[0] = command & MAIN_COMMAND_MASK;
  cmd.command_length = 1;

  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    pending_commands_.push(cmd);
  }
}

void BLDCDriverBoard::SendGetCommand(uint8_t motor_index,
                                     COMMAND_TYPE command) {
  Command cmd;
  cmd.command_bytes[0] = command & MAIN_COMMAND_MASK |
                         (motor_index == 1 ? MOTOR_INDEX_BIT : 0) | GET_CMD_BIT;
  cmd.command_length = 1;

  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    pending_commands_.push(cmd);
  }
}

void BLDCDriverBoard::SendGetCommand(uint8_t motor_index, COMMAND_TYPE command,
                                     COMMAND_TYPE subcommand) {
  Command cmd;
  cmd.command_bytes[0] = command & MAIN_COMMAND_MASK |
                         (motor_index == 1 ? MOTOR_INDEX_BIT : 0) | GET_CMD_BIT;
  cmd.command_bytes[1] = subcommand;
  cmd.command_length = 2;

  {
    std::lock_guard<std::mutex> guard(commands_mutex_);
    pending_commands_.push(cmd);
  }
}
