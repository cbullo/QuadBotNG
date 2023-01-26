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
#include <iomanip>
#include <iostream>

#include "fmt/core.h"
#include "motor.h"
#include "src/libs/communication/binary_stream.h"

const int BLDCDriverBoard::kReadBuffer;

BLDCDriverBoard::BLDCDriverBoard() {}

void BLDCDriverBoard::UpdateConfig(const YAML::Node &config) {
  usb_address_ = config["controller_address"].as<std::string>();
  // linearization_offset_ = config["linearization_offset"].as<uint8_t>();
  // linearization_factors_ =
}

bool BLDCDriverBoard::Connect() {
  sync_state_ = SyncState::kNotSynced;

  std::chrono::seconds pause_time(1);
  std::this_thread::sleep_for(pause_time);
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
  config.c_cc[VMIN] = 0;
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

  SyncState sync_state_ = SyncState::kNotSynced;
  std::cout << "Disconnected" << std::endl;
}

bool BLDCDriverBoard::IsConnected() const { return serial_ != -1; }

void BLDCDriverBoard::StartCommunicationThread() {
  read_offset_ = 0;
  msg_part_ = ExpectedMessagePart::kHeaderPart;
  expected_msg_bytes_ = MESSAGE_HEADER_SIZE;

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

  // std::cout << "Sending command: "
  //           << "0x" << std::setfill('0') << std::setw(2) << std::right
  //           << std::hex << (int)to_process.command_bytes[0] << std::endl;
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
        std::cout << "EAGAIN" << std::endl;
        continue;
      } else {
        HandleCommunicationError(
            fmt::format("SendCommand write() returned error {}: {}", errno,
                        strerror(errno)));
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
      HandleCommunicationError(
          fmt::format("DiscardReceive: ioctl() returned error {}: {}", errno,
                      strerror(errno)));
    }
    if (value == 0) {
      break;
    }
    bytes_read = read(serial_, serial_read_buffer_,
                      std::min(value, BLDCDriverBoard::kReadBuffer));

    if (bytes_read == -1) {
      if (errno == EAGAIN) {
        break;
      } else {
        HandleCommunicationError(
            fmt::format("DiscardReceive read() returned error {}: {}", errno,
                        strerror(errno)));
        break;
      }
    } else {
      std::cout << "Discarding " << std::dec << bytes_read
                << " bytes, first byte: " << std::hex
                << (int)serial_read_buffer_[0] << std::endl;
    }
  }
}

uint8_t BLDCDriverBoard::CalculateReplySize(uint8_t header_byte) {
  return GetArgumentLength(header_byte);
}

uint8_t BLDCDriverBoard::CalculateDataStreamSize(uint8_t header_byte) {
  // std::cout << "CalculatedDataStreamSize - header: " << std::hex <<
  // header_byte;
  uint8_t data_fields = header_byte & MESSAGE_SUBTYPE_MASK;
  uint8_t data_size = 0;

  // std::cout << "CalculatedDataStreamSize - data_fields: " << std::hex <<
  // data_fields;

  if (data_fields & DATA_STREAM_ANGLE_BIT) {
    data_size += sizeof(uint16_t);
    // } else if (data_fields & DATA_STREAM_VELOCITY_BIT) {
    //   data_size += sizeof(float);
  }

  if (data_fields & DATA_STREAM_TEMPERATURE_BIT) {
    data_size += sizeof(uint16_t);
  }
  data_size *= 2;  // Always return values for both motors

  // std::cout << "CalculatedDataStreamSize - data_size: " << std::hex <<
  // data_size << std::endl;
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
    case MESSAGE_TYPE_DATA_STREAM: {
      auto ret = CalculateDataStreamSize(bytes[0]);
      // std::cout << "CalculatedDataStreamSize: " << std::dec << (int)ret
      //           << std::endl;
      return ret;
    }
    case MESSAGE_TYPE_SYNC:
    case MESSAGE_TYPE_PING:
      return 0;
    default:
      std::cout << "Unknown message type: " << std::hex << (int)bytes[0]
                << std::endl;
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
      HandleCommunicationError(
          fmt::format("ProcessReceive ioctl() returned error {}: {}", errno,
                      strerror(errno)));
      return;
    }
    // std::cout << "Bytes available: " << value;
    if (value == 0) {
      // const std::string no_data = "No data available";
      // for (int i = 0; i < no_data.size(); ++i) {
      //   std::cout << "\b";
      // }
      break;
    }
    bytes_read = read(
        fd, serial_read_buffer_ + read_offset_,
        std::min(value, static_cast<int>(expected_msg_bytes_ - read_offset_)));
    if (bytes_read > 0) {
      // std::cout << "ProcessReceive received: " << std::endl;
      // for (int i = 0; i < bytes_read; i++) {
      //   std::cout << std::setfill('0') << std::setw(2) << std::right
      //             << std::hex
      //             << (unsigned int)(serial_read_buffer_[read_offset_ + i]);
      // }
    } else {
      break;
    }

    if (bytes_read == -1) {
      if (errno == EAGAIN) {
        break;
      } else {
        HandleCommunicationError(
            fmt::format("ProcessReceive read() returned error {}: {}", errno,
                        strerror(errno)));
        return;
      }
    } else {
      read_offset_ += bytes_read;

      // std::cout << "Debug 00: " << (int)read_offset_ << " " << (int)msg_part_
      //           << " " << expected_msg_bytes_ << std::endl;
      while (read_offset_ == expected_msg_bytes_) {
        // std::cout << "Debug 0: " << (int)read_offset_ << " " <<
        // (int)msg_part_
        //           << " " << expected_msg_bytes_ << std::endl;
        switch (msg_part_) {
          case ExpectedMessagePart::kHeaderPart:
            expected_msg_bytes_ += ProcessHeader(serial_read_buffer_[0]);
            // std::cout << "Header processed" << std::endl;
            // std::cout << "Debug: " << (int)read_offset_ << " " <<
            // (int)msg_part_
            //           << " " << expected_msg_bytes_ << std::endl;
            msg_part_ = ExpectedMessagePart::kStaticPart;
            break;
          case ExpectedMessagePart::kStaticPart:
            expected_msg_bytes_ += ProcessStaticData(serial_read_buffer_);
            // std::cout << "Static processed" << std::endl;
            // std::cout << "Debug: " << (int)read_offset_ << " " <<
            // (int)msg_part_
            //           << " " << expected_msg_bytes_ << std::endl;
            msg_part_ = ExpectedMessagePart::kDynamicPart;
            break;
          case ExpectedMessagePart::kDynamicPart:
            ProcessMessage(serial_read_buffer_, read_offset_);
            // std::cout << "Dynamic processed" << std::endl;
            // std::cout.flush();
            // std::cout << "Debug: " << (int)read_offset_ << " " <<
            // (int)msg_part_
            //           << " " << expected_msg_bytes_ << std::endl;
            // std::cout.flush();
            read_offset_ = 0;
            msg_part_ = ExpectedMessagePart::kHeaderPart;
            expected_msg_bytes_ = MESSAGE_HEADER_SIZE;
            break;
          default:
            std::cout << "Something's wrong??" << std::endl;
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
        HandleCommunicationError(fmt::format(
            "Unexpected message type received.\nExpected: {} Received: {}",
            cmd.command_bytes[0], header_byte));
      }
      return 0;
    }
    case MESSAGE_TYPE_DATA_STREAM:
      return 0;
    case MESSAGE_TYPE_STRING:
      return 1;
  }
}

inline uint16_t normalizeAngle(int32_t in, uint16_t period) {
  while (in < 0) {
    in += period;
  }

  while (in >= period) {
    in -= period;
  }

  return in;
}

void BLDCDriverBoard::SendSetupVariables() {
  if (motors_[0]) {
    std::cout << "1" << motors_[0] << std::endl;
    SendSetCommandAndSub(0, CMD_SENSOR_LINEARIZATION, SCMD_OFFSET,
                         motors_[0]->sensor_linearization_offset_,
                         static_cast<uint8_t>(0));
    for (int i = 0; i < motors_[0]->sensor_linearization_coeffs_.size(); ++i) {
      SendSetCommandAndSub(0, CMD_SENSOR_LINEARIZATION, SCMD_FACTOR,
                           static_cast<uint8_t>(i),
                           motors_[0]->sensor_linearization_coeffs_[i]);
    }

    SendSetCommandAndSub(
        0, CMD_SENSOR, SCMD_SENS_ELEC_OFFSET,
        static_cast<int32_t>(motors_[0]->electric_zero_angle_));
    // SendSetCommand(0, CMD_MOTION_TYPE, static_cast<uint8_t>(1));
  }
  if (motors_[1]) {
    std::cout << "2" << std::endl;
    std::cout << motors_[1]->GetName() << std::endl;
    SendSetCommandAndSub(1, CMD_SENSOR_LINEARIZATION, SCMD_OFFSET,
                         motors_[1]->sensor_linearization_offset_,
                         static_cast<uint8_t>(0));
    std::cout << "2.1" << std::endl;
    for (int i = 0; i < motors_[1]->sensor_linearization_coeffs_.size(); ++i) {
      SendSetCommandAndSub(1, CMD_SENSOR_LINEARIZATION, SCMD_FACTOR,
                           static_cast<uint8_t>(i),
                           motors_[1]->sensor_linearization_coeffs_[i]);
    }
    SendSetCommandAndSub(
        1, CMD_SENSOR, SCMD_SENS_ELEC_OFFSET,
        static_cast<int32_t>(motors_[1]->electric_zero_angle_));
    // SendSetCommand(1, CMD_MOTION_TYPE, static_cast<uint8_t>(1));
  }

  SendSetCommand(0, CMD_STATE, static_cast<uint8_t>(1));
}

void BLDCDriverBoard::ProcessMessage(const uint8_t *message, int message_size) {
  // Make sure we're getting data that we're expecting
  switch (sync_state_) {
    case SyncState::kNotSynced:
    case SyncState::kSyncingPause:
      std::cout << "Not expected" << std::endl;
      // We shouldn't bother calling this if not synced.
      assert(false);
      return;
    case SyncState::kSyncing:
      std::cout << "Processing SyncState::kSyncing" << std::endl;
      if (message_size != 1 ||
          message[0] != (MESSAGE_TYPE_SYNC | MESSAGE_SUBTYPE_SYNC)) {
        std::cout << "try handle error" << std::endl;
        HandleCommunicationError(fmt::format(
            "ProcessMessage: expected sync message, received: {}", message[0]));
        return;
      }
      SetState(SyncState::kSynced);
      SendSetupVariables();

      std::cout << "Setup variables sent" << std::endl;
      std::cout.flush();
      BumpTimeout(1000);
      return;
    case SyncState::kSynced:
      if ((message[0] & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_PING) {
        std::cout << "Ping received" << std::endl;
        BumpTimeout(1000);
        return;
      }
      break;
  }

  BumpTimeout(1000);
  if ((message[0] & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_GET_REPLY) {
    Command read_command;
    {
      std::lock_guard<std::mutex> guard(commands_mutex_);
      read_command = sent_get_commands_.front();
    }
    read_command.callback->ReadArgs(message, message_size);
    {
      std::lock_guard<std::mutex> guard(reply_mutex_);
      replied_commands_queue_.push_back(read_command);
    }
  } else if ((message[0] & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_STRING) {
    uint8_t string_length = message[1];
    std::string received_string(reinterpret_cast<const char *>(message + 2),
                                string_length);
    switch (message[0] & MESSAGE_SUBTYPE_MASK) {
      case STRING_MESSAGE_TYPE_ERROR:
        std::cout << "ERROR: " << received_string << std::endl;
        HandleCommunicationError(received_string);
        break;
      case STRING_MESSAGE_TYPE_WARNING:
        std::cout << "WARNING: " << received_string << std::endl;
        break;
      case STRING_MESSAGE_TYPE_INFO:
        std::cout << "INFO: " << received_string << std::endl;
        break;
    }
  } else if ((message[0] & MESSAGE_TYPE_MASK) == MESSAGE_TYPE_DATA_STREAM) {
    if (message[0] & DATA_STREAM_ANGLE_BIT) {
      //std::cout << *reinterpret_cast<const uint16_t *>(&message[1]);
      //std::cout << " ";
      std::cout << *reinterpret_cast<const uint16_t *>(&message[3]);
      std::cout << std::endl;
    }
  } else {
    HandleCommunicationError(
        fmt::format("Unknown message type: {}", message[0]));
  }
}

void BLDCDriverBoard::ProcessLoop() {
  std::chrono::seconds pause_time(2);
  std::this_thread::sleep_for(pause_time);
  communication_deadline_ = std::chrono::system_clock::now();
  BumpTimeout(3000);

  while (IsConnected()) {
    switch (sync_state_) {
      case SyncState::kNotSynced:
        SendSync();

        std::cout << "Sync sent" << std::endl;

        DiscardReceive();
        // ProcessSend();
        break;
      case SyncState::kSyncingPause:
        DiscardReceive();
        ProcessSend();
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
    CheckTimeout();
  }
}

void BLDCDriverBoard::ResetBoard() {
  HardwareReset();
  // std::cout << "ResetBoard called" << std::endl;
}

void BLDCDriverBoard::HardwareReset() {
  int RTS_flag = TIOCM_RTS;
  ioctl(serial_, TIOCMBIS, &RTS_flag);  // Set RTS pin
  usleep(10000);
  ioctl(serial_, TIOCMBIC, &RTS_flag);  // Clear RTS pin
}

void BLDCDriverBoard::Tick() {
  if (error_state_) {
    // spdlog::error("\"{}\" BLDCDriverBoard error: {}", name_, error_message_);
    // spdlog::warning("Resetting {}", name_);
    ResetBoard();
  }
  RelayMessages();
}

void BLDCDriverBoard::RelayMessages() {
  std::vector<Command> replies;
  {
    std::lock_guard<std::mutex> guard(reply_mutex_);
    replies.insert(replies.begin(), replied_commands_queue_.begin(),
                   replied_commands_queue_.end());
  }
  for (auto &reply : replies) {
    (*reply.callback)();
  }
}

void BLDCDriverBoard::SendSync() {
  SendCommand(CMD_SYNC);
  BumpTimeout(1000);
  SetState(SyncState::kSyncingPause);
}

void BLDCDriverBoard::CheckTimeout() {
  if (std::chrono::system_clock::now() >= communication_deadline_) {
    // HandleCommunicationError(
    //     fmt::format("CheckTimeout: communication timeout"));
  }
}

void BLDCDriverBoard::BumpTimeout(int ms) {
  communication_deadline_ =
      std::chrono::system_clock::now() + std::chrono::milliseconds{ms};
  // std::cout << "Bump" << std::endl;
}

void BLDCDriverBoard::SetState(SyncState new_state) {
  sync_state_ = new_state;
  std::cout << "Board new state: " << static_cast<int>(new_state) << std::endl;
}

void BLDCDriverBoard::SendCommand(COMMAND_TYPE command) {
  Command cmd;
  cmd.command_bytes[0] = command & MAIN_COMMAND_MASK;
  cmd.command_length = 1;

  // std::cout << "Pushing command: " << std::hex << cmd.command_bytes[0]
  //           << std::endl;

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

void BLDCDriverBoard::HandleCommunicationError(const std::string &message) {
  bool print_error = error_message_ != message;
  error_message_ = message;
  error_state_ = true;

  if (print_error) {
    std::cout << "Communication error: " << message << std::endl;
  }

  ResetBoard();
}