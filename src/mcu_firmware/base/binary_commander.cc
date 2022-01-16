#include "binary_commander.h"

BinaryCommander::BinaryCommander(Stream& serial) : bytes_to_read_(1) {
  com_port = &serial;
}

BinaryCommander::BinaryCommander() : bytes_to_read_(1) {}

// void Commander::add(char id, CommandCallback onCommand, char* label ){
//   call_list[call_count] = onCommand;
//   call_ids[call_count] = id;
//   call_label[call_count] = label;
//   call_count++;
// }

void BinaryCommander::run() {
  if (!com_port) return;
  run(*com_port, eol);
}

void BinaryCommander::run(Stream& serial) {
  Stream* tmp = com_port;  // save the serial instance
  char eol_tmp = this->eol;
  this->eol = eol;
  com_port = &serial;

  while (serial.available()) {
    // get the new byte:
    int ch = serial.read();
    received_cmd[rec_cnt++] = (uint8_t)ch;

    --bytes_to_read;

    if (rec_cnt == 1) {
      bytes_to_read = GetMessageLength(received_cmd[0]) - 1;
    }

    bool is_get = IsGetMessage(received_cmd[0]);
    if (!is_get) {
      bytes_to_read += GetMessageLength(received_cmd[0]);
    }

    // end of user input
    if (bytes_to_read == 0) {
      // execute the user command
      run(received_cmd);

      // reset the command buffer
      rec_cnt = 0;
      bytes_to_read = 1;
    }

    if (rec_cnt >=
        MAX_COMMAND_LENGTH) {  // prevent buffer overrun if message is too long
      rec_cnt = 0;
      bytes_to_read = 1;
      SendString(ST::kError, "CMTL");
    }
  }

  com_port = tmp;  // reset the instance to the internal value
  this->eol = eol_tmp;
}

void BinaryCommander::run(uint8_t* user_input) {
  auto cmd = user_input[0];
  auto main_command = cmd & MAIN_COMMAND_MASK;

  if (call_list[main_command]) {
    call_list[main_command](user_input);
  }
}

void ProcessStateCommand(ControllerState new_state) {
  switch (controller_state) {
    case ControllerState::PRE_INIT:
      if (cmd[1] == 'I') {
        controller_state = ControllerState::INIT;
        return;
      }
      break;
    case ControllerState::INIT:
      break;
    case ControllerState::STOPPED:
      if (cmd[1] == 'R') {
        motors[0].enable();
        motors[1].enable();
        controller_state = ControllerState::RUNNING;
        return;
      }
      break;
    case ControllerState::RUNNING:
      if (cmd[1] == 'S') {
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
  uint8_t msg_length = GetMessageLength(cmd);
  uint8_t sub_cmd = GetCommand(user_command[1]);
  uint8_t sequence = 0;
  if (is_get) {
    seq = user_command[msg_length];
    sub_cmd &= GET_CMD_BIT;
  }
  // check if there is a subcommand or not
  switch (cmd) {
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
            SendReply(motor->voltage_limit, sequence);
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
            SendReply(motor->current_limit, sequence);
          }
          break;
        case SCMD_LIM_VEL:  // velocity limit
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          } else {
            SendReply(motor->velocity_limit, sequence);
          }
          break;
        default:
          SendString(ST::kError, "ULCM");
          break;
      }
      break;

    case CMD_MOTION_DOWNSAMPLE:
      if (!is_get) {
        uint16_t value =
            *(reinterpret_cast<uint16_t*>(user_command + msg_length));
        motor->motion_downsample = value;
      } else {
        SendReply(static_cast<uint16_t>(motor->motion_downsample), sequence);
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
        SendReply(static_cast<uint8_t>(motor->controller), sequence);
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
        SendReply(static_cast<uint8_t>(motor->torque_controller), sequence);
      }
      break;
    case CMD_STATUS:
      // enable/disable
      if (!is_get) {
        uint8_t value =
            *(reinterpret_cast<uint8_t*>(user_command + msg_length));
        value ? motor->enable() : motor->disable();
      } else {
        SendReply(static_cast<uint8_t>(motor->enabled), sequence);
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
            SendReply(static_cast<uint8_t>(motor->foc_modulation), sequence);
          }
          break;
        case SCMD_PWMMOD_CENTER:  // centered modulation
          if (!is_get) {
            uint8_t value =
                *(reinterpret_cast<uint8_t*>(user_command + msg_length));
            motor->modulation_centered = value;
          } else {
            SendReply(static_cast<uint8_t>(motor->modulation_centered),
                      sequence);
          }
          break;
        default:
          SendString(ST::kError, "UPCM");
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
        SendReply(_isset(motor->phase_resistance) ? motor->phase_resistance : 0,
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
            SendReply(motor->sensor_offset, sequence);
          }
          break;
        case SCMD_SENS_ELEC_OFFSET:  // electrical zero offset - not
                                     // suggested to touch
          if (!is_get) {
            float value =
                *(reinterpret_cast<float*>(user_command + msg_length));
            motor->zero_electric_angle = value;

          } else {
            SendReply(motor->zero_electric_angle, sequence);
          }
          break;
        default:
          SendString(ST::kError, "USCM");
          break;
      }
      break;
    case CMD_SENSOR_LINEARIZATION:
      switch (sub_cmd) {
        case SCMD_OFFSET:
          uint8_t value = *(reinterpret_cast<uint8_t*>(
              user_command + msg_length + sizeof(uint8_t)));
          sensor[motor_index]->linearization_.offset_ = value;
        case SCMD_FACTOR:
          float coeff_index =
              *(reinterpret_cast<uint8_t*>(user_command + msg_length));
          uint8_t value = *(reinterpret_cast<uint8_t*>(
              user_command + msg_length + sizeof(uint8_t)));
          sensor[motor_index]->linearization_.coeffs_[coeff_index] = value;
          break;
      }
    case CMD_STATE: {
      uint8_t value = *(reinterpret_cast<uint8_t*>(user_command + msg_length));
      if (!is_get) {
        ProcessStateCommand(static_cast<>(value));
      } else {
        SendReply(controller_state, sequence);
      }
    } break;
    case CMD_AVAILABILITY:
      if (!is_get) {
        availability[motor_index] = value;
      } else {
        return availability[motor_index];
      }
      break;

#if FOC_USE_MONITORING
    case CMD_MONITOR:  // get current values of the state variables
      printVerbose(F("Monitor | "));
      switch (sub_cmd) {
        case SCMD_GET:  // get command
          switch ((uint8_t)value) {
            case 0:  // get target
              printVerbose(F("target: "));
              println(motor->target);
              break;
            case 1:  // get voltage q
              printVerbose(F("Vq: "));
              println(motor->voltage.q);
              break;
            case 2:  // get voltage d
              printVerbose(F("Vd: "));
              println(motor->voltage.d);
              break;
            case 3:  // get current q
              printVerbose(F("Cq: "));
              println(motor->current.q);
              break;
            case 4:  // get current d
              printVerbose(F("Cd: "));
              println(motor->current.d);
              break;
            case 5:  // get velocity
              printVerbose(F("vel: "));
              println(motor->shaft_velocity);
              break;
            case 6:  // get angle
              printVerbose(F("angle: "));
              println(motor->shaft_angle);
              break;
            case 7:  // get all states
              printVerbose(F("all: "));
              print(motor->target);
              print(";");
              print(motor->voltage.q);
              print(";");
              print(motor->voltage.d);
              print(";");
              print(motor->current.q);
              print(";");
              print(motor->current.d);
              print(";");
              print(motor->shaft_velocity);
              print(";");
              println(motor->shaft_angle);
              break;
            default:
              printError();
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:
          printVerbose(F("downsample: "));
          if (!GET) motor->monitor_downsample = value;
          println((int)motor->monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor->monitor_variables = (uint8_t)0;
          println(F("clear"));
          break;
        case SCMD_SET:
          if (!GET) motor->monitor_variables = (uint8_t)0;
          for (int i = 0; i < 7; i++) {
            if (isSentinel(user_command[value_index + i])) break;
            if (!GET)
              motor->monitor_variables |= (user_command[value_index + i] - '0')
                                          << (6 - i);
            print((user_command[value_index + i] - '0'));
          }
          println("");
          break;
        default:
          printError();
          break;
      }
      break;
#endif
    default:  // unknown cmd
      SendString(ST::kError, "UCMD");
      break;
  }
}

void BinaryCommander::pid(PIDController* pid, char* user_cmd) {
  char cmd = user_cmd[0];
  bool is_get = IsGetMessage(user_cmd[0]);
  uint8_t msg_length = GetPIDSubmessageLength(cmd);
  uint8_t sequence = 0;
  if (is_get) {
    seq = user_command[msg_length];
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
      SendString(ST::kError, "UPID");
      return;
  }
  if (!is_get) {
    *variable = value;
  } else {
    SendReply(*variable, sequence);
  }
}

void BinaryCommander::lpf(LowPassFilter* lpf, char* user_cmd) {
  char cmd = user_cmd[0];
  bool is_get = IsGetMessage(user_cmd[0]);
  uint8_t msg_length = GetMessageLength(cmd);
  uint8_t sequence = 0;
  if (is_get) {
    seq = user_command[msg_length];
  }
  float value = *(reinterpret_cast<float*>(user_cmd + msg_length));
  float* variable = nullptr;

  switch (cmd) {
    case SCMD_LPF_TF:  // Tf value change
      variable = &lpf->Tf break;
    default:
      SendString(ST::kError, "ULPF");
      return;
  }
  if (!is_get) {
    *variable = value;
  } else {
    SendReply(*variable, sequence);
  }
}
