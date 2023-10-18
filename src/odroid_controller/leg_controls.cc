#include "leg_controls.h"

#include <unistd.h>

#include "leg.h"
#include "log_helper.h"
#include "pid_utils.h"

float next_read_delta_ = 0.f;

bool ThetaGammaZLegControl::Process(Leg& leg, float dt) {
  // auto t = leg.GetTheta();
  // auto g = leg.GetGamma();
  // auto z = leg.GetAngleZ();

  // next_read_delta_ -= dt;
  // if (next_read_delta_ <= 0.f) {
  //   std::cout << "T:" << t << " G:" << g << " Z:" << z << std::endl;
  //   next_read_delta_ = 0.5f;
  // }
  // return true;

  const double kUpdateDelta = 0.005;

  // std::cout << dt << std::endl;

  delay_ -= dt;
  if (delay_ <= 0.f) {
    double az = leg.GetAngleZ();
    double theta = leg.GetTheta();
    double gamma = leg.GetGamma();

    float tau_theta =
        UpdatePIDControl(state_.theta_pid_state, leg.GetThetaPIDConfig(),
                         kUpdateDelta, state_.theta_setpoint, theta, false);
    float tau_gamma =
        UpdatePIDControl(state_.gamma_pid_state, leg.GetGammaPIDConfig(),
                         kUpdateDelta, state_.gamma_setpoint, gamma, false);
    float tau_z = UpdatePIDControl(state_.z_pid_state, leg.GetZPIDConfig(),
                                   kUpdateDelta, state_.z_setpoint, az, false);

    float steering_i = (0.5 * tau_theta + 0.5 * tau_gamma);
    float steering_o = (0.5 * tau_theta - 0.5 * tau_gamma);
    float steering_z = tau_z /*- 0.75f * tau_gamma*/;

    steering_i = std::clamp(steering_i, -8.f, 8.f);
    steering_o = std::clamp(steering_o, -8.f, 8.f);
    steering_z = std::clamp(steering_z, -8.f, 8.f);

    // std::cout << leg.GetAngleI() << " " << leg.GetAngleO() << " " <<
    // std::cout << "z: " << az << " st z: " << state_.z_setpoint
    //           << " ctr z: " << leg.GetMotorZ()->GetDirection() * steering_z
    //           << std::endl;

    // std::cout << "i: " << leg.GetAngleI() << " ctr i: "
    //           << steering_i
    //           //           << " ctr z: " << leg.GetMotorZ()->GetDirection() *
    //           //           steering_z
    //           << std::endl;

    // std::cout << theta << " " << state_.theta_setpoint << " " << gamma << "
    // "
    //           << state_.gamma_setpoint << " " << steering_i << " " <<
    //           steering_o
    //           << std::endl;

    // steering_i = -4.f;
    // steering_o = -4.f;
    // steering_z = 0.f;

    // std::cout << gamma << " " << state_.gamma_setpoint << " " << steering_i
    //           << " " << steering_o << std::endl;

    leg.GetMotorI()->GetController()->SendSetCommand(
        leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(leg.GetMotorI()->GetDirection() * 512.0 *
                             steering_i));
    leg.GetMotorO()->GetController()->SendSetCommand(
        leg.GetMotorO()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(leg.GetMotorO()->GetDirection() * 512.0 *
                             steering_o));
    leg.GetMotorZ()->GetController()->SendSetCommand(
        leg.GetMotorZ()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(leg.GetMotorZ()->GetDirection() * 512.0 *
                             steering_z));

    delay_ = kUpdateDelta;
  }

  return true;
}

void CalibrationLegControl::SetCalibrationMode(
    CalibrationLegControl::CalibrationMode mode) {
  std::cout << "Calibration mode set: " << (int)mode << std::endl;
  calibration_mode_ = mode;
  step_ = 0;
  SetRunning(false);
}

void CalibrationLegControl::SetMotorIndex(int index) {
  if (0 <= index && index <= 3) {
    motor_index_ = index;
    // step_ = 0;
    // SetRunning(false);
    // OutputMode(calibration_mode_, motor_index_);
  }
}

void CalibrationLegControl::SendSetupData(Leg& leg) {
  auto* motor = GetMotor(leg, motor_index_);
  if (!motor) {
    return;
  }
  switch (calibration_mode_) {
    case CalibrationLegControl::LinearizationReading:
      motor->GetController()->SendSetCommandAndSub(
          motor->GetIndex(), CMD_SENSOR_LINEARIZATION, SCMD_OFFSET,
          static_cast<uint8_t>(0), static_cast<uint8_t>(0));
      for (int i = 0; i < motor->sensor_linearization_coeffs_.size(); ++i) {
        motor->GetController()->SendSetCommandAndSub(
            motor->GetIndex(), CMD_SENSOR_LINEARIZATION, SCMD_FACTOR,
            static_cast<uint8_t>(i), static_cast<uint8_t>(0));
      }

      motor->GetController()->SendSetCommand(motor->GetIndex(), CMD_MOTOR_PHASE,
                                             static_cast<int16_t>(0),
                                             static_cast<int16_t>(0));
      step_ = 0;
      break;
    case CalibrationLegControl::LinearizationValidation:
      motor->GetController()->SendSetCommand(0, CMD_STATE,
                                             static_cast<uint8_t>(3));
      break;
    case CalibrationLegControl::MotorVoltage:
      motor->GetController()->SendSetCommand(0, CMD_STATE,
                                             static_cast<uint8_t>(3));

      step_ = 0;
      break;
    case CalibrationLegControl::ElectricZero:
      std::cout << "sending el zero" << std::endl;
      motor->GetController()->SendSetCommandAndSub(
          motor->GetIndex(), CMD_SENSOR, SCMD_SENS_ELEC_OFFSET,
          static_cast<int16_t>(0));

      // motor->GetController()->SendSetCommand(motor->GetIndex(),
      // CMD_MOTOR_PHASE,
      //                                        static_cast<int16_t>(0),
      //                                        static_cast<int16_t>(512 *
      //                                        5));

      motor->GetController()->SendSetCommand(0, CMD_STATE,
                                             static_cast<uint8_t>(3));

      step_ = 0;
      avg_zero_angle_ = 0.f;
      break;
  }
}

bool CalibrationLegControl::Process(Leg& leg, float dt) {
  const double kUpdateDelta = 0.005;

  if (!IsRunning()) {
    return false;
  }

  switch (calibration_mode_) {
    case CalibrationLegControl::CalibrationMode::LinearizationReading: {
      next_read_delta_ -= dt;
      if (next_read_delta_ <= 0.f) {
        auto* motor = GetMotor(leg, motor_index_);
        if (motor) {
          auto angle = motor->GetRawAngle();
          // std::cout << "\e[2K\r" << step_ << ", " << angle;
          std::cout << step_ << ", " << angle << std::endl;
          raw_angle_ = angle;
        }
        next_read_delta_ = 0.1f;
      }
      break;
    }
    case CalibrationLegControl::CalibrationMode::ElectricZero: {
      auto* motor = GetMotor(leg, motor_index_);
      if (!motor) {
        return false;
      }
      bool do_step = false;
      if (step_ < 4096 * 7) {
        if (step_ % 4096 == 0) {
          if (next_read_delta_ <= 0.f) {
            std::cout << "Set next" << std::endl;
            next_read_delta_ = 3.0f;
            motor->GetController()->SendSetCommand(
                motor->GetIndex(), CMD_MOTOR_PHASE, static_cast<int16_t>(0),
                static_cast<int16_t>(8 * 512));
            break;
          } else {
            next_read_delta_ -= dt;
            if (next_read_delta_ <= 0.f) {
              auto angle = motor->GetRawAngle();
              int zero_angle = ((angle * 7) % 4096) / 7;
              std::cout << "Zero angle: " << zero_angle << std::endl;
              step_ += 4;
              avg_zero_angle_ += static_cast<float>(zero_angle) / 7;
            }
            break;
          }
        }

        if (next_read_delta_ <= 0.f) {
          motor->GetController()->SendSetCommand(
              motor->GetIndex(), CMD_MOTOR_PHASE,
              static_cast<int16_t>(step_ % 4096),
              static_cast<int16_t>(5 * 512));
          next_read_delta_ = 0.001f;
        } else {
          next_read_delta_ -= dt;
          if (next_read_delta_ <= 0.f) {
            step_ += 4;
          }
        }
      } else {
        motor->GetController()->SendSetCommand(
            motor->GetIndex(), CMD_MOTOR_PHASE, static_cast<int16_t>(0),
            static_cast<int16_t>(0));
        motor->GetController()->SendSetCommand(0, CMD_STATE,
                                               static_cast<uint8_t>(2));
        std::cout << "Avg. zero angle: " << avg_zero_angle_ << std::endl;

        SetRunning(false);
      }
    } break;

    case CalibrationLegControl::CalibrationMode::LinearizationValidation: {
      auto* motor = GetMotor(leg, motor_index_);
      if (!motor) {
        return false;
      }
      bool do_step = false;
      if (step_ < 4096 * 7) {
        if (step_ % 64 == 0) {
          if (next_read_delta_ <= 0.f) {
            std::cout << "Set next" << std::endl;
            next_read_delta_ = 0.01f;
            motor->GetController()->SendSetCommand(
                motor->GetIndex(), CMD_MOTOR_PHASE,
                static_cast<int16_t>((motor->GetDirection() < 0)
                                         ? (4095 - step_ % 4096)
                                         : (step_ % 4096)),
                static_cast<int16_t>(8 * 512));
            break;
          } else {
            next_read_delta_ -= dt;
            if (next_read_delta_ <= 0.f) {
              auto angle = 4095 - motor->GetRawAngle();
              if (step_ == 0) {
                zero_angle_ = angle;
              }

              int set_angle = step_ / 7;
              int measured = angle - zero_angle_;
              if (measured < 0) {
                measured += 4096;
              }

              double fang = measured * Motor::kAS5600ToRadians;
              double fset_ang = set_angle * Motor::kAS5600ToRadians;

              std::cout << "Set: " << set_angle << " Measured: " << measured
                        << " Error: " << fabs(DirectionDistance(fang, fset_ang))
                        << std::endl;
              step_ += 8;
            }
            break;
          }
        }

        if (next_read_delta_ <= 0.f) {
          motor->GetController()->SendSetCommand(
              motor->GetIndex(), CMD_MOTOR_PHASE,
              static_cast<int16_t>((motor->GetDirection() < 0)
                                       ? (4095 - step_ % 4096)
                                       : (step_ % 4096)),
              static_cast<int16_t>(5 * 512));
          next_read_delta_ = 0.01f;
        } else {
          next_read_delta_ -= dt;
          if (next_read_delta_ <= 0.f) {
            step_ += 8;
          }
        }
      } else {
        motor->GetController()->SendSetCommand(
            motor->GetIndex(), CMD_MOTOR_PHASE, static_cast<int16_t>(0),
            static_cast<int16_t>(0));
        motor->GetController()->SendSetCommand(0, CMD_STATE,
                                               static_cast<uint8_t>(2));

        SetRunning(false);
      }
    } break;
    case CalibrationLegControl::CalibrationMode::MotorVoltage: {
      auto* motor = GetMotor(leg, motor_index_);
      if (!motor) {
        return false;
      }

      delay_ -= dt;

      if (delay_ <= 0.f) {
        float voltage = 8.0 * voltage_control_;
        int16_t value =
            static_cast<int16_t>(motor->GetDirection() * 512.0 * voltage);

        // std::cout << "voltage: " << voltage << " value: " << value <<
        // std::endl;
        std::cout << motor->GetRawAngle() << " " << motor->GetAccumulatedAngle()
                  << std::endl;
        motor->GetController()->SendSetCommand(motor->GetIndex(),
                                               CMD_MOTOR_VOLTAGE, value);
        delay_ = kUpdateDelta;
      }
    } break;
  }

  return true;
}

const float kDriveVoltage = 12.f;

bool Drive(Steering::DisabledT dir, float current, PIDState& state,
           const PIDParams& params, float dt, float& tau_theta, bool debug) {
  // leg.GetMotorI()->GetController()->SendSetCommand(
  //     leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE, kDriveVoltage);
  // leg.GetMotorO()->GetController()->SendSetCommand(
  //     leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE, kDriveVoltage);
  state.last_target_set = false;
  tau_theta = 0.f;
  return true;
}

bool Drive(Steering::PositiveT dir, float current, PIDState& state,
           const PIDParams& params, float dt, float& tau_theta, bool debug) {
  tau_theta =
      UpdatePIDControl(state, params, dt, state.last_target_set ? state.last_target + 0.02 * dt * M_PI : current, current, debug);
  if (state.time_since_value_stable > 1.f) {
    return true;
  }
  return false;
}

bool Drive(Steering::NegativeT dir, float current, PIDState& state,
           const PIDParams& params, float dt, float& tau_theta, bool debug) {
  tau_theta =
      UpdatePIDControl(state, params, dt, state.last_target_set ? state.last_target - 0.02 * dt * M_PI : current, current, debug);
  if (state.time_since_value_stable > 1.f) {
    return true;
  }
  return false;
}

bool Drive(float target, float current, PIDState& state,
           const PIDParams& params, float dt, float& tau_theta, bool debug) {
  tau_theta = UpdatePIDControl(state, params, dt, target, current, debug);
  if (state.time_since_value_stable > 1.f &&
      fabsf(target - state.approx_running_mean) < 0.1f) {
    return true;
  }
  return false;
}

// bool InitializationLegControl::DriveTheta(Leg& leg, float value, float dt,
//                                           float& i, float& o) {
//   // leg.GetMotorI()->GetController()->SendSetCommand(
//   //     leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE, kDriveVoltage);
//   // leg.GetMotorO()->GetController()->SendSetCommand(
//   //     leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE, kDriveVoltage);

//   float theta = leg.GetTheta();
//   float tau_theta = UpdatePIDControl(state_.theta_pid_state,
//                                      leg.theta_pd_config, dt, value, theta);

//   i += tau_theta;
//   o += tau_theta;
// }

void InitializationLegControl::ChangeStage(InitializationStage new_stage) {
  std::cout << "New stage: " << static_cast<int>(new_stage) << std::endl;
  stage_ = new_stage;
  state_.Reset();
}

bool InitializationLegControl::Process(Leg& leg, float dt) {
  LegSteering steering;

  const double kUpdateDelta = 0.005;

  delay_ -= dt;
  // if (delay_ <= 0.f) {
    switch (stage_) {
      case InitializationStage::PreInitialization:
        std::cout << "PreInitialization ";
        break;
      case InitializationStage::DriveToMinGamma:
        std::cout << "DriveToMinGamma ";
        break;
      case InitializationStage::DriveToMinZ:
        std::cout << "DriveToMinZ ";
        break;
      case InitializationStage::DriveToNextThetaRev:
        std::cout << "DriveToNextThetaRev ";
        break;
      case InitializationStage::DriveToSafeZ:
        std::cout << "DriveToSafeZ ";
        break;
      case InitializationStage::FindMinZ:
        std::cout << "FindMinZ ";
        break;
    };

    switch (stage_) {
      case InitializationStage::PreInitialization: {
        locked_z_ = leg.GetAngleZ();
        locked_theta_ = leg.GetTheta();
        locked_gamma_ = leg.GetGamma();
      } break;

      case InitializationStage::DriveToMinGamma: {
        if (DriveTo(leg, state_, locked_theta_, Steering::Negative,
                    locked_z_, leg.GetState(), dt, steering)) {
          // if (!VerifyPosition<Gamma>(leg)) {
          //   return false;
          // }
          // locked_theta_ = leg.GetTheta();
          // locked_z_ = leg.GetAngleZ();
          // locked_gamma_ = leg.GetGamma();
          // ChangeStage(InitializationStage::FindMinZ);
          ChangeStage(InitializationStage::Done);
        }
      } break;

      case InitializationStage::FindMinZ: {
        if (DriveTo(leg, state_, locked_theta_, locked_gamma_,
                    Steering::Negative, leg.GetState(), kUpdateDelta,
                    steering)) {
          // leg.GetMotorZ()->SetCurrentAngle(leg.GetMaxZ());
          leg.UpdateZOffset(leg.GetMaxZ());
          ChangeStage(InitializationStage::DriveToSafeZ);
          // if (!VerifyPosition<Z>(leg)) {

          //   DriveTo(locked_theta_, leg.GetMinGamma(), Direction::Negative) {

          //   }
          // }
        }
      } break;

      case InitializationStage::DriveToMinZ: {
        if (DriveTo(leg, state_,
                    locked_theta_ + current_theta_rev_ * (2 / 9.f) * M_PI,
                    locked_gamma_, Steering::Negative, leg.GetState(),
                    kUpdateDelta, steering)) {
          measured_z_angle_[current_theta_rev_] = leg.GetAngleZ();
          if (current_theta_rev_ < 9) {
            ChangeStage(InitializationStage::DriveToSafeZ);
          } else {
            ChangeStage(InitializationStage::PreInitialization);
            // SetActive(false);
          };
          // if (!VerifyPosition<Z>(leg)) {

          //   DriveTo(locked_theta_, leg.GetMinGamma(), Direction::Negative) {

          //   }
          // }
        }
      } break;

      case InitializationStage::DriveToSafeZ: {
        if (DriveTo(leg, state_,
                    locked_theta_ + current_theta_rev_ * (2 / 9.f) * M_PI,
                    locked_gamma_, leg.GetInitSafeZ(), leg.GetState(),
                    kUpdateDelta, steering)) {
          ChangeStage(InitializationStage::DriveToNextThetaRev);
          // if (!VerifyPosition<Z>(leg)) {

          //   DriveTo(locked_theta_, leg.GetMinGamma(), Direction::Negative) {

          //   }
          // }
        }
        // steering.steering_z = -steering.steering_z;
      } break;
      case InitializationStage::DriveToNextThetaRev: {
        if (DriveTo(leg, state_,
                    locked_theta_ + current_theta_rev_ * (2 / 9.f) * M_PI,
                    locked_gamma_, leg.GetInitSafeZ(), leg.GetState(),
                    kUpdateDelta, steering)) {
          ChangeStage(InitializationStage::DriveToMinZ);

          ++current_theta_rev_;
          if (current_theta_rev_ < 9) {
            ChangeStage(InitializationStage::DriveToMinZ);
          } else {
            ChangeStage(InitializationStage::PreInitialization);
            // SetActive(false);
          };
        }
        // steering.steering_z = -steering.steering_z;
      } break;
      default:
        return true;
    }

    steering.steering_i = std::clamp(steering.steering_i, -12.f, 12.f);
    steering.steering_o = std::clamp(steering.steering_o, -12.f, 12.f);
    steering.steering_z = std::clamp(steering.steering_z, -12.f, 12.f);

    leg.GetMotorI()->GetController()->SendSetCommand(
        leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(leg.GetMotorI()->GetDirection() * 512.0 *
                             steering.steering_i));
    leg.GetMotorO()->GetController()->SendSetCommand(
        leg.GetMotorO()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(leg.GetMotorO()->GetDirection() * 512.0 *
                             steering.steering_o));
    leg.GetMotorZ()->GetController()->SendSetCommand(
        leg.GetMotorZ()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(leg.GetMotorZ()->GetDirection() * 512.0 *
                             steering.steering_z));
    delay_ = kUpdateDelta;

    //usleep(100000);

    // std::cout << steering.steering_i << " " << steering.steering_o << " "
    //           << steering.steering_z << std::endl;
  //}
  return true;
}