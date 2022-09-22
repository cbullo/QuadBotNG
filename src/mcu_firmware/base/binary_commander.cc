#include "binary_commander.h"

#include "src/libs/communication/binary_stream.h"

FOCMotor* GetMotor(uint8_t index);
CustomMagneticSensorI2C* GetSensor(uint8_t index);
extern uint8_t availability[2];

BinaryCommander::BinaryCommander() : bytes_to_read(1) {}

void BinaryCommander::BumpTimeout() {
  micros_timeout_ = (_micros() + 500) % 1000;
}

void BinaryCommander::CheckAndSendPing() {
  if (!sync_sent_) {
    return;
  }
  if (_micros() % 1000 < micros_timeout_) {
    SendPing();
  }
}

void BinaryCommander::run() {
  // CheckAndSendPing();

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
    }

    bool is_get = IsGetMessage(received_cmd[0]);
    if (!is_get) {
      bytes_to_read += GetArgumentLength(received_cmd[0]);
    }

    // end of user input
    if (bytes_to_read == 0) {
      // execute the user command
      process(static_cast<uint8_t*>(received_cmd));

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

extern float temperature[2];

static uint8_t next_data = 0;
void BinaryCommander::SendDataStream() {
  if (Serial.availableForWrite() >= 9) {
    // uint8_t bits[3] = {DATA_STREAM_ANGLE_BIT, DATA_STREAM_VELOCITY_BIT,
    //                    DATA_STREAM_TEMPERATURE_BIT};
    // float data[2] = {GetSensor(0)->getAngle(),
    //                  GetSensor(1)->getAngle()};
    float data;
    uint8_t msg = MESSAGE_TYPE_DATA_STREAM;
    switch (next_data) {
      case 0:
        msg |= DATA_STREAM_ANGLE_BIT;
        Serial.write(&msg, 1);
        data = GetSensor(0)->getSensorAngle();
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
        data = GetSensor(1)->getSensorAngle();
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
        break;
      case 1:
        msg |= DATA_STREAM_VELOCITY_BIT;
        Serial.write(&msg, 1);
        data = GetSensor(0)->getVelocity();
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
        data = GetSensor(1)->getVelocity();
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
        break;
      case 2:
        msg |= DATA_STREAM_TEMPERATURE_BIT;
        Serial.write(&msg, 1);
        data = temperature[0];
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
        data = temperature[1];
        Serial.write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
        break;
    }
    next_data = (next_data + 1) % 3;
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

void ProcessStateCommand(ControllerState new_state) {
  switch (controller_state) {
    case ControllerState::PRE_INIT:
      if (new_state == ControllerState::INIT) {
        controller_state = ControllerState::INIT;
        return;
      }
      break;
    case ControllerState::INIT:
      break;
    case ControllerState::STOPPED:
      if (new_state == ControllerState::RUNNING) {
        GetMotor(0)->enable();
        GetMotor(1)->enable();
        controller_state = ControllerState::RUNNING;
        return;
      }
      break;
    case ControllerState::RUNNING:
      if (new_state == ControllerState::STOPPED) {
        controller_state = ControllerState::STOPPED;
        return;
      }
      break;
    case ControllerState::ERROR:
      break;
  }
}

void BinaryCommander::process(uint8_t* user_command) {
  bool is_get = IsGetMessage(user_command[0]);
  uint8_t motor_index = GetMotorIndex(user_command[0]);
  FOCMotor* motor = GetMotor(motor_index);

  // parse command letter
  uint8_t cmd = GetCommand(user_command[0]);
  uint8_t msg_length = GetCommandLength(cmd);
  uint8_t sub_cmd = GetCommand(user_command[1]);
  uint8_t sequence = 0;
  if (is_get) {
    uint8_t seq = user_command[msg_length];
    sub_cmd &= GET_CMD_BIT;
  }
  // check if there is a subcommand or not
  switch (cmd) {
    case CMD_SYNC:
      _delay(2000);
      SendSync();
      break;
    case CMD_V_PID:  //
      if (sub_cmd == SCMD_LPF_TF)
        lpf(&motor->LPF_velocity, &user_command[1]);
      else
        pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:  //
      if (sub_cmd == SCMD_LPF_TF)
        lpf(&motor->LPF_angle, &user_command[1]);
      else
        pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:  //
      switch (sub_cmd) {
        case SCMD_LIM_VOLT:  // voltage limit change
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->voltage_limit = value;
#if FOC_USE_CURRENT_SENSE
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
#endif
            // change velocity pid limit if in voltage mode and no phase
            // resistance set
            if (!_isset(motor->phase_resistance) &&
                motor->torque_controller == TorqueControlType::voltage)
              motor->PID_velocity.limit = value;
          } else {
            SendReply(cmd, motor->voltage_limit, sequence);
          }
          break;
        case SCMD_LIM_CURR:  // current limit
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->current_limit = value;
            // if phase resistance specified or the current control is on set
            // the current limit to the velocity PID
            if (_isset(motor->phase_resistance) ||
                motor->torque_controller != TorqueControlType::voltage)
              motor->PID_velocity.limit = value;
          } else {
            SendReply(cmd, motor->current_limit, sequence);
          }
          break;
        case SCMD_LIM_VEL:  // velocity limit
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          } else {
            SendReply(cmd, motor->velocity_limit, sequence);
          }
          break;
        default:
          SendString(STRING_MESSAGE_TYPE_ERROR, "ULCM");
          break;
      }
      break;

    case CMD_MOTION_DOWNSAMPLE:
      if (!is_get) {
        uint16_t value =
            *(reinterpret_cast<uint16_t*>(user_command + msg_length));
        motor->motion_downsample = value;
      } else {
        SendReply(cmd, static_cast<uint16_t>(motor->motion_downsample),
                  sequence);
      }
      break;
    case CMD_MOTION_TYPE:
      // change control type
      if (!is_get) {
        uint8_t value =
            *(reinterpret_cast<uint8_t*>(user_command + msg_length));
        if (value >= 0 && value < 5) {
          motor->controller = (MotionControlType)value;
        }
      } else {
        SendReply(cmd, static_cast<uint8_t>(motor->controller), sequence);
      }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      if (!is_get) {
        uint8_t value =
            *(reinterpret_cast<uint8_t*>(user_command + msg_length));
        if (value >= 0 && value < 3) {
          motor->torque_controller = (TorqueControlType)value;
        }
      } else {
        SendReply(cmd, static_cast<uint8_t>(motor->torque_controller),
                  sequence);
      }
      break;
    case CMD_STATUS:
      // enable/disable
      if (!is_get) {
        uint8_t value =
            *(reinterpret_cast<uint8_t*>(user_command + msg_length));
        value ? motor->enable() : motor->disable();
      } else {
        SendReply(cmd, static_cast<uint8_t>(motor->enabled), sequence);
      }
      break;
    case CMD_PWMMOD:
      // PWM modulation change
      switch (sub_cmd) {
        case SCMD_PWMMOD_TYPE:
          if (!is_get) {
            uint8_t value =
                *(reinterpret_cast<uint8_t*>(user_command + msg_length));
            motor->foc_modulation = (FOCModulationType)value;
          } else {
            SendReply(cmd, static_cast<uint8_t>(motor->foc_modulation),
                      sequence);
          }
          break;
        case SCMD_PWMMOD_CENTER:  // centered modulation
          if (!is_get) {
            uint8_t value =
                *(reinterpret_cast<uint8_t*>(user_command + msg_length));
            motor->modulation_centered = value;
          } else {
            SendReply(cmd, static_cast<uint8_t>(motor->modulation_centered),
                      sequence);
          }
          break;
        default:
          SendString(STRING_MESSAGE_TYPE_ERROR, "UPCM");
          break;
      }
      break;
    case CMD_RESIST:
      // enable/disable
      if (!is_get) {
        float value = *(reinterpret_cast<float*>(user_command + msg_length));
        motor->phase_resistance = value;

        if (motor->torque_controller == TorqueControlType::voltage) {
          motor->voltage_limit = motor->current_limit * value;
          motor->PID_velocity.limit = motor->current_limit;
        }
      } else {
        SendReply(cmd,
                  _isset(motor->phase_resistance) ? motor->phase_resistance : 0,
                  sequence);
      }
      break;
    case CMD_SENSOR:
      // Sensor zero offset
      switch (sub_cmd) {
        case SCMD_SENS_MECH_OFFSET:  // zero offset
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->sensor_offset = value;
          } else {
            SendReply(cmd, motor->sensor_offset, sequence);
          }
          break;
        case SCMD_SENS_ELEC_OFFSET:  // electrical zero offset - not
                                     // suggested to touch
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->zero_electric_angle = value;

          } else {
            SendReply(cmd, motor->zero_electric_angle, sequence);
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
          uint8_t value = *(reinterpret_cast<uint8_t*>(
              user_command + msg_length + sizeof(uint8_t)));
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
    case CMD_STATE: {
      uint8_t value = *(reinterpret_cast<uint8_t*>(user_command + msg_length));
      if (!is_get) {
        ProcessStateCommand(static_cast<ControllerState>(value));
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

void BinaryCommander::pid(PIDController* pid, uint8_t* user_cmd) {
  char cmd = user_cmd[0];
  bool is_get = IsGetMessage(user_cmd[0]);
  uint8_t msg_length = GetPIDSubcommandLength(cmd);
  uint8_t sequence = 0;
  if (is_get) {
    sequence = user_cmd[msg_length];
  }
  float value = *(reinterpret_cast<float*>(user_cmd + msg_length));
  float* variable = nullptr;

  switch (cmd) {
    case SCMD_PID_P:  // P gain change
      variable = &pid->P;
      break;
    case SCMD_PID_I:  // I gain change
      variable = &pid->I;
      break;
    case SCMD_PID_D:  // D gain change
      variable = &pid->D;
      break;
    case SCMD_PID_RAMP:  //  ramp change
      variable = &pid->output_ramp;
      break;
    case SCMD_PID_LIM:  //  limit change
      variable = &pid->limit;
      break;
    default:
      SendString(STRING_MESSAGE_TYPE_ERROR, "UPID");
      return;
  }
  if (!is_get) {
    *variable = value;
  } else {
    SendReply(cmd, *variable, sequence);
  }
}

void BinaryCommander::lpf(LowPassFilter* lpf, uint8_t* user_cmd) {
  char cmd = user_cmd[0];
  bool is_get = IsGetMessage(user_cmd[0]);
  uint8_t msg_length = GetCommandLength(cmd);
  uint8_t sequence = 0;
  if (is_get) {
    sequence = user_cmd[msg_length];
  }
  float value = *(reinterpret_cast<float*>(user_cmd + msg_length));
  float* variable = nullptr;

  switch (cmd) {
    case SCMD_LPF_TF:  // Tf value change
      variable = &lpf->Tf;
      break;
    default:
      SendString(STRING_MESSAGE_TYPE_ERROR, "ULPF");
      return;
  }
  if (!is_get) {
    *variable = value;
  } else {
    SendReply(cmd, *variable, sequence);
  }
}

void BinaryCommander::SendString(uint8_t string_type, const char* message) {
  Serial.write(MESSAGE_TYPE_STRING & string_type);
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

void BinaryCommander::SendReply(uint8_t cmd, float var, uint8_t sequence) {
  Serial.write(cmd & MESSAGE_TYPE_GET_REPLY);
  Serial.write(reinterpret_cast<uint8_t*>(&var), sizeof(float));
  Serial.write(&sequence, 1);
  BumpTimeout();
}

void BinaryCommander::SendSync() {
  Serial.write((uint8_t)(MESSAGE_TYPE_SYNC | MESSAGE_SUBTYPE_SYNC));
  sync_sent_ = true;
  BumpTimeout();
}

void BinaryCommander::SendPing() {
  Serial.write((uint8_t)(MESSAGE_TYPE_PING));
  BumpTimeout();
}