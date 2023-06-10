#include "binary_commander.h"

#include "src/libs/communication/binary_stream.h"
#include "src/mcu_firmware/base/simple_foc/time_utils.h"

// FOCMotor* GetMotor(uint8_t index);
CustomMagneticSensorI2C* GetSensor(uint8_t index);
extern uint8_t availability[2];
extern BLDCMotor motors[2];

BinaryCommander::BinaryCommander() : bytes_to_read(1) {}

void BinaryCommander::BumpTimeout() { micros_timeout_ = _micros() + 500000; }

void BinaryCommander::CheckAndSendPing() {
  if (!sync_sent_) {
    return;
  }
  if (_micros() > micros_timeout_) {
    SendPing();
  }
}

void BinaryCommander::run() {
  CheckAndSendPing();

  while (Serial.available()) {
    // get the new byte:
    int ch = Serial.read();
    if (ch < 0) {
      return;
    }

    received_cmd[rec_cnt++] = (uint8_t)ch;
    // Serial.print(ch);

    --bytes_to_read;

    if (rec_cnt == 1) {
      bytes_to_read = GetCommandLength(received_cmd[0]) - 1;
      bool is_get = IsGetMessage(received_cmd[0]);
      if (!is_get) {
        bytes_to_read += GetArgumentLength(received_cmd[0]);
      }
    }

    // end of user input
    if (bytes_to_read == 0) {
      // execute the user command
      process(received_cmd);

      // reset the command buffer
      rec_cnt = 0;
      bytes_to_read = 1;
    }

    if (rec_cnt >=
        MAX_COMMAND_SIZE) {  // prevent buffer overrun if message is too long
      rec_cnt = 0;
      bytes_to_read = 1;
      SendString(STRING_MESSAGE_TYPE_ERROR, "CMTL");
    }
  }

  if (sync_sent_) {
    SendDataStream();
  }
}

extern uint16_t temperature[2];

static uint8_t next_data = 0;
void BinaryCommander::SendDataStream() {
  if (Serial.availableForWrite() >= 5) {
    // uint8_t bits[3] = {DATA_STREAM_ANGLE_BIT, DATA_STREAM_VELOCITY_BIT,
    //                    DATA_STREAM_TEMPERATURE_BIT};
    // float data[2] = {GetSensor(0)->getAngle(),
    //                  GetSensor(1)->getAngle()};

    if (GetSensor(0)->getLastAngleReadingTime() < 0 ||
        GetSensor(1)->getLastAngleReadingTime() < 0) {
      return;
    }

    uint8_t msg = MESSAGE_TYPE_DATA_STREAM;
    switch (next_data) {
      case 0: {
        uint16_t data;
        msg |= DATA_STREAM_ANGLE_BIT;
        Serial.write(&msg, 1);
        data = GetSensor(0)->getMechanicalAngle();
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));
        data = GetSensor(1)->getMechanicalAngle();
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));
      } break;
      // case 1:
      //   msg |= DATA_STREAM_VELOCITY_BIT;
      //   Serial.write(&msg, 1);
      //   data = GetSensor(0)->getVelocity();
      //   Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
      //   data = GetSensor(1)->getVelocity();
      //   Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
      //   break;
      case 1: {
        uint16_t data;
        msg |= DATA_STREAM_TEMPERATURE_BIT;
        Serial.write(&msg, 1);
        data = temperature[0] >> 8;
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));
        data = temperature[1] >> 8;
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));
      } break;
    }
    // next_data = (next_data + 1) % 2;
    BumpTimeout();
  }
}

// void BinaryCommander::run(uint8_t* user_input) {
//   auto cmd = user_input[0];
//   auto main_command = cmd & MAIN_COMMAND_MASK;

//   if (call_list[main_command]) {
//     call_list[main_command](user_input);
//   }
// }

extern ControllerState controller_state;

#define MOTOR_AVAILABLE 0x01
#define SENSOR_AVAILABLE 0x02

void ProcessStateCommand(BinaryCommander* cmd, ControllerState new_state) {
  switch (controller_state) {
    case ControllerState::PRE_INIT:
      cmd->SendString(STRING_MESSAGE_TYPE_INFO, "CSPI");
      if (new_state == ControllerState::INIT) {
        controller_state = ControllerState::INIT;
      }
      break;
    case ControllerState::INIT:
      cmd->SendString(STRING_MESSAGE_TYPE_INFO, "CSI");
      break;
    case ControllerState::STOPPED:
      cmd->SendString(STRING_MESSAGE_TYPE_INFO, "CSS");
      if (new_state == ControllerState::RUNNING) {
        if (availability[0] & MOTOR_AVAILABLE) {
          motors[0].enable();
        }
        if (availability[1] & MOTOR_AVAILABLE) {
          motors[1].enable();
        }
        controller_state = ControllerState::RUNNING;
      }
      break;
    case ControllerState::RUNNING:
      cmd->SendString(STRING_MESSAGE_TYPE_INFO, "CSR");
      if (new_state == ControllerState::STOPPED) {
        controller_state = ControllerState::STOPPED;
        if (availability[0] & MOTOR_AVAILABLE) {
          motors[0].disable();
        }
        if (availability[1] & MOTOR_AVAILABLE) {
          motors[1].disable();
        }
      }
      break;
    case ControllerState::ERROR:
      cmd->SendString(STRING_MESSAGE_TYPE_INFO, "CSE");
      break;
  }
}

void BinaryCommander::process(uint8_t* user_command) {
  bool is_get = IsGetMessage(user_command[0]);
  uint8_t motor_index = GetMotorIndex(user_command[0]);

  // parse command letter
  uint8_t cmd = GetCommand(user_command[0]);
  uint8_t msg_length = GetCommandLength(cmd);
  uint8_t sub_cmd = GetCommand(user_command[1]);
  uint8_t sequence = 0;
  if (is_get) {
    // uint8_t seq = user_command[msg_length];
    // sub_cmd &= GET_CMD_BIT;
  }
  // check if there is a subcommand or not
  switch (cmd) {
    case CMD_SYNC:
      _delay(2000);
      // SendString(STRING_MESSAGE_TYPE_INFO, "SYNC");
      SendSync();
      break;
    // case CMD_V_PID:  //
    //   SendString(STRING_MESSAGE_TYPE_INFO, "????");
    //   if (sub_cmd == SCMD_LPF_TF)
    //     lpf(&motors[motor_index].LPF_velocity, &user_command[1]);
    //   else
    //     pid(&motors[motor_index].PID_velocity, &user_command[1]);
    //   break;
    // case CMD_A_PID:  //
    //   SendString(STRING_MESSAGE_TYPE_INFO, "????");
    //   if (sub_cmd == SCMD_LPF_TF) {
    //     lpf(&motors[motor_index].LPF_angle, &user_command[1]);
    //   } else {
    //     pid(&motors[motor_index].P_angle, &user_command[1]);
    //   }
    //   break;
    case CMD_LIMITS:  //
      switch (sub_cmd) {
        case SCMD_LIM_VOLT:  // voltage limit change
          if (!is_get) {
            int16_t value =
                *(reinterpret_cast<int16_t*>(user_command + msg_length));
            motors[motor_index].voltage_limit = value;
            // change velocity pid limit if in voltage mode and no phase
            // resistance set
            // if (!_isset(motors[motor_index].phase_resistance) &&
            //     motors[motor_index].torque_controller ==
            //         TorqueControlType::voltage)
            //   motors[motor_index].PID_velocity.limit = value;
          } else {
            SendReply(cmd, motors[motor_index].voltage_limit, sequence);
          }
          break;
        // case SCMD_LIM_CURR:  // current limit
        //   if (!is_get) {
        //     float value =
        //         *(reinterpret_cast<float*>(user_command + msg_length));
        //     motors[motor_index].current_limit = value;
        //     // if phase resistance specified or the current control is on set
        //     // the current limit to the velocity PID
        //     if (_isset(motors[motor_index].phase_resistance) ||
        //         motors[motor_index].torque_controller !=
        //             TorqueControlType::voltage)
        //       motors[motor_index].PID_velocity.limit = value;
        //   } else {
        //     SendReply(cmd, motors[motor_index].current_limit, sequence);
        //   }
        //   break;
        // case SCMD_LIM_VEL:  // velocity limit
        //   if (!is_get) {
        //     float value =
        //         *(reinterpret_cast<float*>(user_command + msg_length));
        //     motors[motor_index].velocity_limit = value;
        //     motors[motor_index].P_angle.limit = value;
        //   } else {
        //     SendReply(cmd, motors[motor_index].velocity_limit, sequence);
        //   }
        //   break;
        default:
          SendString(STRING_MESSAGE_TYPE_ERROR, "ULCM");
          break;
      }
      break;

    // case CMD_MOTION_DOWNSAMPLE:
    //   if (!is_get) {
    //     uint16_t value =
    //         *(reinterpret_cast<uint16_t*>(user_command + msg_length));
    //     motors[motor_index].motion_downsample = value;
    //   } else {
    //     SendReply(cmd,
    //               static_cast<uint16_t>(motors[motor_index].motion_downsample),
    //               sequence);
    //   }
    //   break;
    case CMD_MOTION_TYPE:
      // change control type
      if (!is_get) {
        uint8_t value =
            *(reinterpret_cast<uint8_t*>(user_command + msg_length));
        if (value >= 0 && value < 5) {
          motors[motor_index].controller = (MotionControlType)value;
        }
      } else {
        SendReply(cmd, static_cast<uint8_t>(motors[motor_index].controller),
                  sequence);
      }
      break;
    case CMD_PWMMOD:
      // PWM modulation change
      switch (sub_cmd) {
        // case SCMD_PWMMOD_TYPE:
        //   if (!is_get) {
        //     uint8_t value =
        //         *(reinterpret_cast<uint8_t*>(user_command + msg_length));
        //     motors[motor_index].foc_modulation = (FOCModulationType)value;
        //   } else {
        //     SendReply(cmd,
        //               static_cast<uint8_t>(motors[motor_index].foc_modulation),
        //               sequence);
        //   }
        //   break;
        case SCMD_PWMMOD_CENTER:  // centered modulation
          if (!is_get) {
            uint8_t value =
                *(reinterpret_cast<uint8_t*>(user_command + msg_length));
            motors[motor_index].modulation_centered = value;
          } else {
            SendReply(
                cmd,
                static_cast<uint8_t>(motors[motor_index].modulation_centered),
                sequence);
          }
          break;
        default:
          SendString(STRING_MESSAGE_TYPE_ERROR, "UPCM");
          break;
      }
      break;
    // case CMD_RESIST:
    //   // enable/disable
    //   if (!is_get) {
    //     float value = *(reinterpret_cast<float*>(user_command + msg_length));
    //     motors[motor_index].phase_resistance = value;

    //     if (motors[motor_index].torque_controller ==
    //         TorqueControlType::voltage) {
    //       motors[motor_index].voltage_limit =
    //           motors[motor_index].current_limit * value;
    //       motors[motor_index].PID_velocity.limit =
    //           motors[motor_index].current_limit;
    //     }
    //   } else {
    //     SendReply(cmd,
    //               _isset(motors[motor_index].phase_resistance)
    //                   ? motors[motor_index].phase_resistance
    //                   : 0,
    //               sequence);
    //   }
    //   break;
    case CMD_MOTOR_VOLTAGE:
      if (!is_get) {
        int16_t value = *reinterpret_cast<int16_t*>(user_command + msg_length);

        // char str[10];
        // int v = static_cast<int>(value);
        // sprintf(str, "%d", v);
        // SendString(STRING_MESSAGE_TYPE_INFO, str);

        motors[motor_index].voltage = value;
        motors[motor_index].controller = MotionControlType::voltage;
      } else {
        // SendReply(cmd,
        //           _isset(motors[motor_index].phase_resistance) ?
        //           motors[motor_index].phase_resistance : 0, sequence);
      }
      break;
    case CMD_MOTOR_PHASE:
      if (!is_get) {
        int16_t phase = *reinterpret_cast<int16_t*>(user_command + msg_length);
        int16_t voltage = *reinterpret_cast<int16_t*>(
            user_command + msg_length + sizeof(int16_t));

        // char str[10];
        // int v = static_cast<int>(voltage);
        // sprintf(str, "%d", v);
        // SendString(STRING_MESSAGE_TYPE_INFO, str);

        motors[motor_index].phase = phase;
        motors[motor_index].voltage = voltage;
        motors[motor_index].controller = MotionControlType::direct_phase;
      } else {
        // SendReply(cmd,
        //           _isset(motors[motor_index].phase_resistance) ?
        //           motors[motor_index].phase_resistance : 0, sequence);
      }
      break;
    case CMD_SENSOR:
      // Sensor zero offset
      switch (sub_cmd) {
        // case SCMD_SENS_MECH_OFFSET:  // zero offset
        //   if (!is_get) {
        //     float value =
        //         *(reinterpret_cast<float*>(user_command + msg_length));
        //     motors[motor_index].sensor_offset = value;
        //   } else {
        //     SendReply(cmd, motors[motor_index].sensor_offset, sequence);
        //   }
        //   break;
        case SCMD_SENS_ELEC_OFFSET:  // electrical zero offset - not
                                     // suggested to touch
          if (!is_get) {
            int16_t value =
                *(reinterpret_cast<int16_t*>(user_command + msg_length));
            motors[motor_index].zero_electric_angle = value;

          } else {
            SendReply(cmd, motors[motor_index].zero_electric_angle, sequence);
          }
          break;
        default:
          SendString(STRING_MESSAGE_TYPE_ERROR, "USCM");
          break;
      }
      break;
    case CMD_SENSOR_LINEARIZATION:
      switch (sub_cmd) {
        case SCMD_OFFSET: {
          uint8_t value =
              *(reinterpret_cast<uint8_t*>(user_command + msg_length));

          // This is hacky - swapped wires
          uint8_t swap_wires = *(reinterpret_cast<uint8_t*>(
              user_command + msg_length + sizeof(uint8_t)));
          if (swap_wires) {
            auto prvB = motors[motor_index].pwmB;
            motors[motor_index].pwmB = motors[motor_index].pwmC;
            motors[motor_index].pwmC = prvB;
          }

          GetSensor(motor_index)->linearization_.offset_ = value;
          break;
        }
        case SCMD_FACTOR: {
          uint8_t coeff_index =
              *(reinterpret_cast<uint8_t*>(user_command + msg_length));
          uint8_t value = *(reinterpret_cast<uint8_t*>(
              user_command + msg_length + sizeof(uint8_t)));
          GetSensor(motor_index)->linearization_.coeffs_[coeff_index] = value;
          break;
        }
      }
      break;
    case CMD_STATE: {
      uint8_t value = *(reinterpret_cast<uint8_t*>(user_command + msg_length));
      if (!is_get) {
        char s[6];
        s[0] = 'C';
        s[1] = 'P';
        s[2] = 'S';
        s[3] = 'C';
        s[4] = 'A' + value;
        s[5] = 0;
        SendString(STRING_MESSAGE_TYPE_INFO, s);
        ProcessStateCommand(this, static_cast<ControllerState>((int)value));
      } else {
        SendReply(cmd, static_cast<uint8_t>(controller_state), sequence);
      }
    } break;
    case CMD_AVAILABILITY: {
      uint8_t value = *(reinterpret_cast<uint8_t*>(user_command + msg_length));
      if (!is_get) {
        availability[motor_index] = value;
      } else {
        SendReply(cmd, static_cast<uint8_t>(availability[motor_index]),
                  sequence);
      }
      break;
    }
    default:  // unknown cmd
      SendString(STRING_MESSAGE_TYPE_ERROR, "UCMD");
      break;
  }
}

// void BinaryCommander::pid(PIDController* pid, uint8_t* user_cmd) {
//   SendString(STRING_MESSAGE_TYPE_INFO, "????");
//   char cmd = user_cmd[0];
//   bool is_get = IsGetMessage(user_cmd[0]);
//   uint8_t msg_length = GetPIDSubcommandLength(cmd);
//   uint8_t sequence = 0;
//   if (is_get) {
//     sequence = user_cmd[msg_length];
//   }
//   float value = *(reinterpret_cast<float*>(user_cmd + msg_length));
//   volatile float* variable = nullptr;

//   switch (cmd) {
//     case SCMD_PID_P:  // P gain change
//       variable = &pid->P;
//       break;
//     case SCMD_PID_I:  // I gain change
//       variable = &pid->I;
//       break;
//     case SCMD_PID_D:  // D gain change
//       variable = &pid->D;
//       break;
//     case SCMD_PID_RAMP:  //  ramp change
//       variable = &pid->output_ramp;
//       break;
//     case SCMD_PID_LIM:  //  limit change
//       variable = &pid->limit;
//       break;
//     default:
//       SendString(STRING_MESSAGE_TYPE_ERROR, "UPID");
//       return;
//   }
//   if (!is_get) {
//     *variable = value;
//   } else {
//     SendReply(cmd, *variable, sequence);
//   }
// }

// void BinaryCommander::lpf(LowPassFilter* lpf, uint8_t* user_cmd) {
//   SendString(STRING_MESSAGE_TYPE_INFO, "????");
//   char cmd = user_cmd[0];
//   bool is_get = IsGetMessage(user_cmd[0]);
//   uint8_t msg_length = GetCommandLength(cmd);
//   uint8_t sequence = 0;
//   if (is_get) {
//     sequence = user_cmd[msg_length];
//   }
//   float value = *(reinterpret_cast<float*>(user_cmd + msg_length));
//   float* variable = nullptr;

//   switch (cmd) {
//     case SCMD_LPF_TF:  // Tf value change
//       variable = &lpf->Tf;
//       break;
//     default:
//       SendString(STRING_MESSAGE_TYPE_ERROR, "ULPF");
//       return;
//   }
//   if (!is_get) {
//     *variable = value;
//   } else {
//     SendReply(cmd, *variable, sequence);
//   }
// }

void BinaryCommander::SendString(uint8_t string_type, const char* message) {
  Serial.write(MESSAGE_TYPE_STRING | string_type);
  uint8_t length = strlen(message);
  Serial.write(&length, 1);
  Serial.write(message, length);
  BumpTimeout();
}

// void BinaryCommander::SendString(StringType level,
//                                  const __FlashStringHelper* message) {
//   com_port->write(MESSAGE_TYPE_STRING);
//   uint8_t length = strlen(message);
//   com_port->write(&length, 1);
//   com_port->write(message, length);
// }

void BinaryCommander::SendSync() {
  Serial.write((uint8_t)(MESSAGE_TYPE_SYNC | MESSAGE_SUBTYPE_SYNC));
  sync_sent_ = true;
  BumpTimeout();
}

void BinaryCommander::SendPing() {
  Serial.write((uint8_t)(MESSAGE_TYPE_PING));
  BumpTimeout();
}