#pragma once

// Odroid -> Driver

#define MAX_COMMAND_SIZE 16

// list of commands
#define CMD_V_PID 0x00              //'V' //!< velocity PID & LPF
#define CMD_A_PID 0x01              //'A' //!< angle PID & LPF
#define CMD_STATUS 0x02             //'E' //!< motor status enable/disable
#define CMD_LIMITS 0x03             //'L' //!< limits current/voltage/velocity
#define CMD_MOTION_TYPE 0x04        //'C' //!< motion control type
#define CMD_MOTION_DOWNSAMPLE 0x05  //'C' //!< motion control type
#define CMD_TORQUE_TYPE 0x06        // 'T' //!< torque control type
#define CMD_SENSOR 0x07             //'S'   //!< sensor offsets
#define CMD_RESIST 0x08             //'R'   //!< motor phase resistance
#define CMD_PWMMOD 0x09             //'W'   //!< pwm modulation
#define CMD_MOTOR_TARGET 0x0A       //" Target get/set

// controller configuration
#define CMD_STATE 0x0B
#define CMD_AVAILABILITY 0x0C
#define CMD_SENSOR_LINEARIZATION 0x0D
#define CMD_SYNC 0x0F

// set to the last command index + 1
#define PRIMARY_COMMANDS_COUNT (CMD_SYNC + 1)
#define MAIN_COMMAND_MASK 0x0F

#define GET_CMD_BIT 0x80
#define MOTOR_INDEX_BIT 0x40

// subcomands
// pid - lpf
#define SCMD_PID_P 0x00     //'P'     //!< PID gain P
#define SCMD_PID_I 0x01     //'I'     //!< PID gain I
#define SCMD_PID_D 0x02     //'D'     //!< PID gain D
#define SCMD_PID_RAMP 0x03  // 'R'  //!< PID ramp
#define SCMD_PID_LIM 0x04   //'L'   //!< PID limit
#define SCMD_LPF_TF 0x05    //'F'    //!< LPF time constant
// limits
#define SCMD_LIM_CURR 0x00  //'C'  //!< Limit current
#define SCMD_LIM_VOLT 0x01  //'U'  //!< Limit voltage
#define SCMD_LIM_VEL 0x02   //'V'   //!< Limit velocity
// sensor
#define SCMD_SENS_MECH_OFFSET 0x00  // 'M'  //!< Sensor offset
#define SCMD_SENS_ELEC_OFFSET 0x01  // 'E'  //!< Sensor electrical zero offset
// monitoring
#define SCMD_DOWNSAMPLE 0x00  //'D'  //!< Monitoring downsample value
#define SCMD_CLEAR 0x01       //'C'       //!< Clear all monitored variables
#define SCMD_GET 0x02         //'G'         //!< Get variable only one value
//#define SCMD_SET 0x03         //'S'         //!< Set variables to be monitored

#define SCMD_PWMMOD_TYPE 0x00    //'T'    //!<< Pwm modulation type
#define SCMD_PWMMOD_CENTER 0x01  //'C'  //!<< Pwm modulation center flag

// linearization
#define SCMD_OFFSET 0x00
#define SCMD_FACTOR 0x01

enum class SyncState {
  kNotSynced = 0,
  kSyncingPause = 1,
  kSyncing = 2,
  kSynced = 3
};

#define COMMAND_TYPE uint8_t

inline uint8_t GetCommand(uint8_t cmd) {
  return cmd & MAIN_COMMAND_MASK;
}

inline bool IsGetMessage(uint8_t cmd) { return cmd & GET_CMD_BIT; }
inline uint8_t GetMotorIndex(uint8_t cmd) {
  return cmd & MOTOR_INDEX_BIT ? 1 : 0;
};

inline uint8_t GetArgumentLength(uint8_t byte) {
  switch (byte & MAIN_COMMAND_MASK) {
    case CMD_V_PID:
      return sizeof(float);
    case CMD_A_PID:
      return sizeof(float);
    case CMD_STATUS:
      return sizeof(uint8_t);
    case CMD_LIMITS:
      return sizeof(float);
    case CMD_MOTION_TYPE:
      return sizeof(uint8_t);
    case CMD_MOTION_DOWNSAMPLE:
      return sizeof(uint16_t);
    case CMD_TORQUE_TYPE:
      return sizeof(uint8_t);
    case CMD_SENSOR:
      return sizeof(float);
    case CMD_RESIST:
      return sizeof(float);
    case CMD_PWMMOD:
      return sizeof(uint8_t);
    case CMD_MOTOR_TARGET:
      return sizeof(float);
    case CMD_STATE:
      return sizeof(uint8_t);
    case CMD_AVAILABILITY:
      return sizeof(uint8_t);
    case CMD_SENSOR_LINEARIZATION:
      return 2 * sizeof(uint8_t);
  }

  return 0;
}

// case SCMD_PID_P:
//       return sizeof(float);
//     case SCMD_PID_I:
//       return sizeof(float);
//     case SCMD_PID_D:
//       return sizeof(float);
//     case SCMD_PID_RAMP:
//       return sizeof(float);
//     case SCMD_PID_LIM:
//       return sizeof(float);

inline uint8_t GetCommandLength(uint8_t byte) {
  switch (byte & MAIN_COMMAND_MASK) {
    case CMD_V_PID:
      return 2 * sizeof(COMMAND_TYPE);
    case CMD_A_PID:
      return 2 * sizeof(COMMAND_TYPE);
    case CMD_STATUS:
      return sizeof(COMMAND_TYPE);
    case CMD_LIMITS:
      return 2 * sizeof(COMMAND_TYPE);
    case CMD_MOTION_TYPE:
      return sizeof(COMMAND_TYPE);
    case CMD_MOTION_DOWNSAMPLE:
      return sizeof(COMMAND_TYPE);
    case CMD_TORQUE_TYPE:
      return sizeof(COMMAND_TYPE);
    case CMD_SENSOR:
      return 2 * sizeof(COMMAND_TYPE);
    case CMD_RESIST:
      return sizeof(COMMAND_TYPE);
    case CMD_PWMMOD:
      return 2 * sizeof(COMMAND_TYPE);
    case CMD_MOTOR_TARGET:
      return sizeof(COMMAND_TYPE);
    case CMD_STATE:
      return sizeof(COMMAND_TYPE);
    case CMD_AVAILABILITY:
      return sizeof(COMMAND_TYPE);
    case CMD_SENSOR_LINEARIZATION:
      return 2 * sizeof(COMMAND_TYPE);
  }

  return 1;
}

inline uint8_t GetPIDSubcommandLength(uint8_t byte) {
  switch (byte & MAIN_COMMAND_MASK) {
    case SCMD_PID_P:
      return sizeof(COMMAND_TYPE);
    case SCMD_PID_I:
      return sizeof(COMMAND_TYPE);
    case SCMD_PID_D:
      return sizeof(COMMAND_TYPE);
    case SCMD_PID_RAMP:
      return sizeof(COMMAND_TYPE);
    case SCMD_PID_LIM:
      return sizeof(COMMAND_TYPE);
  }

  return 1;
}

inline uint8_t GetLPFSubcommandLength(uint8_t byte) {
  switch (byte & MAIN_COMMAND_MASK) {
    case SCMD_LPF_TF:
      return sizeof(COMMAND_TYPE);
  }
  return 1;
}
