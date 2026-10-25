#pragma once
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "src/libs/communication/binary_commands.h"
#include "src/libs/communication/binary_stream.h"

namespace boards {
using Bytes = std::vector<uint8_t>;
template<class T> void Append(Bytes& bytes, T value) {
  static_assert(std::is_integral_v<T> && sizeof(T) <= 2, "wire values must be 8/16 bit integers");
  using U = std::make_unsigned_t<T>;
  U v = static_cast<U>(value);
  for (size_t i = 0; i < sizeof(T); ++i) bytes.push_back((v >> (8*i)) & 255);
}
inline uint16_t U16(const uint8_t* b) { return uint16_t(b[0]) | (uint16_t(b[1]) << 8); }
inline int16_t I16(const uint8_t* b) {
  const int v = U16(b);
  return static_cast<int16_t>(v < 32768 ? v : v - 65536);
}
inline bool SupportsGet(uint8_t command, uint8_t sub = 0) {
  return command == CMD_STATE || command == CMD_AVAILABILITY || command == CMD_MOTION_TYPE ||
         (command == CMD_SENSOR && sub == SCMD_SENS_ELEC_OFFSET) ||
         (command == CMD_LIMITS && sub == SCMD_LIM_VOLT) ||
         (command == CMD_PWMMOD && sub == SCMD_PWMMOD_CENTER);
}
inline size_t CommandSize(uint8_t header) {
  return GetCommandLength(header) + (IsGetMessage(header) ? 0 : GetArgumentLength(header));
}
inline void ValidateCommand(const Bytes& bytes) {
  if (bytes.empty() || bytes.size() != CommandSize(bytes[0]) || bytes.size() > MAX_COMMAND_SIZE)
    throw std::invalid_argument("invalid command payload width");
  if (GetCommand(bytes[0]) == CMD_LIMITS && bytes[1] != SCMD_LIM_VOLT)
    throw std::invalid_argument("limit subcommand unsupported by active firmware");
  if (IsGetMessage(bytes[0]) && !SupportsGet(GetCommand(bytes[0]), bytes.size() > 1 ? bytes[1] : 0))
    throw std::invalid_argument("GET unsupported by active firmware");
}
// GET replies contain a command nibble, payload, and one sequence byte (currently 0).
inline size_t MessageSize(const Bytes& bytes) {
  if (bytes.empty()) return 0;
  const uint8_t h = bytes[0];
  if (h == MESSAGE_SUBTYPE_SYNC || h == MESSAGE_TYPE_PING) return 1;
  switch (h & MESSAGE_TYPE_MASK) {
    case MESSAGE_TYPE_STRING: return bytes.size() < 2 ? 0 : 2 + bytes[1];
    case MESSAGE_TYPE_DATA_STREAM: {
      const auto fields = h & MESSAGE_SUBTYPE_MASK;
      if (fields != DATA_STREAM_ANGLE_BIT && fields != DATA_STREAM_TEMPERATURE_BIT)
        throw std::runtime_error("unsupported telemetry fields");
      return 5;
    }
    case MESSAGE_TYPE_GET_REPLY:
      return 2 + GetArgumentLength(h);
    default: throw std::runtime_error("invalid board frame header");
  }
}
}  // namespace boards
