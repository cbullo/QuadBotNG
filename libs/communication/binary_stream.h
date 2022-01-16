#pragma once

// Driver -> Odroid

#define MAX_MESSAGE_SIZE 16

#define MESSAGE_HEADER_SIZE 1

#define MESSAGE_TYPE_MASK 0xF0
#define MESSAGE_SUBTYPE_MASK 0x0F

#define MESSAGE_TYPE_SYNC 0x00
#define MESSAGE_TYPE_PING 0x20
#define MESSAGE_TYPE_GET_REPLY 0x40
#define MESSAGE_TYPE_DATA_STREAM 0x80
#define MESSAGE_TYPE_STRING 0xC0

#define STRING_MESSAGE_TYPE_ERROR 0x00
#define STRING_MESSAGE_TYPE_WARNING 0x01
#define STRING_MESSAGE_TYPE_INFO 0x02

// Each value defines which data fields are returned in this data pack.
// Motor values are returned for each sensor/motor (even if sensor/motor is
// disabled)

#define DATA_STREAM_ANGLE_BIT 0x01
#define DATA_STREAM_VELOCITY_BIT 0x02
#define DATA_STREAM_TEMPERATURE_BIT 0x04

#define MESSAGE_SUBTYPE_SYNC \
  0x0F  // MESSAGE_TYPE = 0x00, MESSAGE_SUBTYPE = 0x0F

// String message:
// Header (1 byte)
// Size (1 byte)
// Characters (n bytes)
// t - type of string data
// n - length of string
// 1100 tttt nnnn nnnn ...

// Data stream message:
// Header (1 byte)
// Data (length depends on subtype - see list)
// d - type of data stream
// 1000 dddd ...

// Query message:
// Header (1 byte)
// Data (length depends on query type)
// 0100 0000 ....

// Ping message:
// Header (1 byte)
// Sent every 3 sec if no other communication happend before this time passed,
// except when there's 2 sec silence before sync message
// 0010 0000

// Sync message:
// Header (1 byte)
// Slave:
// sync received - 2 sec wait - sync reply
// Master:
// send sync - 1 sec wait - start listening for sync reply
// 0000 1111

// Wrong header read:
// Communication error - reset microcontroller

// 10 seconds silence:
// Connection lost - reset microcontroller