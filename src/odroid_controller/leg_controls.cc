#include "leg_controls.h"

#include "leg.h"
#include "log_helper.h"
#include "pid_utils.h"

bool ThetaGammaZLegControl::Process(Leg& leg, float dt) {
  const double kUpdateDelta = 0.005;

  // std::cout << dt << std::endl;

  delay_ -= dt;
  if (delay_ <= 0.f) {
    double az = leg.GetAngleZ();
    double theta = leg.GetTheta();
    double gamma = leg.GetGamma();

    float tau_theta = UpdatePIDControl(state_.theta_pid_state, leg.theta_pd_config, kUpdateDelta, state_.theta_setpoint, theta);
    float tau_gamma = UpdatePIDControl(state_.gamma_pid_state, leg.gamma_pd_config, kUpdateDelta, state_.gamma_setpoint, gamma);
    float tau_z = UpdatePIDControl(state_.z_pid_state, leg.z_pd_config, kUpdateDelta, state_.z_setpoint, az);

    // std::cout << gamma << std::endl;

    // state_.gamma_setpoint = 0.0;

    double theta_error =
        theta -
        state_.theta_setpoint;  // ClosestAngle(theta, state_.theta_setpoint);
    if (theta_error == 0.0 || theta_error * state_.prev_theta_error < 0.0) {
      state_.i_theta_sum = 0.0;
    }
    state_.i_theta_sum =
        state_.i_theta_sum +
        kUpdateDelta * 0.5 * (theta_error + state_.prev_theta_error);
    double p_term_theta = leg.theta_pd_config.p * (theta_error);
    double d_term_theta = -leg.theta_pd_config.d *
                          (theta_error - state_.prev_theta_error) /
                          kUpdateDelta;
    double i_term_theta = leg.theta_pd_config.i * state_.i_theta_sum;

    double gamma_error = gamma - state_.gamma_setpoint;
    if (gamma_error == 0.0 || gamma_error * state_.prev_gamma_error < 0.0) {
      state_.i_gamma_sum = 0.0;
    }
    state_.i_gamma_sum =
        state_.i_gamma_sum +
        kUpdateDelta * 0.5 * (gamma_error + state_.prev_gamma_error);

    double p_term_gamma = leg.gamma_pd_config.p * (gamma_error);
    double d_term_gamma = -leg.gamma_pd_config.d *
                          (gamma_error - state_.prev_gamma_error) /
                          kUpdateDelta;
    double i_term_gamma = leg.gamma_pd_config.i * state_.i_gamma_sum;

    double tau_theta = p_term_theta + d_term_theta + i_term_theta;
    double tau_gamma = p_term_gamma + d_term_gamma + i_term_gamma;

    // double z_error = ClosestAngle(az, state_.z_setpoint);
    double z_error = az - state_.z_setpoint;
    if (z_error == 0.0 || z_error * state_.prev_z_error < 0.0) {
      state_.i_z_sum = 0.0;
    }
    state_.i_z_sum =
        state_.i_z_sum + kUpdateDelta * 0.5 * (z_error + state_.prev_z_error);
    double p_term_z = leg.z_pd_config.p * (z_error);
    double d_term_z =
        -leg.z_pd_config.d * (z_error - state_.prev_z_error) / kUpdateDelta;
    double i_term_z = leg.z_pd_config.i * state_.i_z_sum;
    double tau_z = p_term_z + d_term_z + i_term_z;

    float steering_i = (0.5 * tau_theta + 0.5 * tau_gamma);
    float steering_o = (0.5 * tau_theta - 0.5 * tau_gamma);
    float steering_z = tau_z /*- 0.75f * tau_gamma*/;

    // float steering_i = 0.5 * tau_theta;
    // float steering_o = 0.5 * tau_theta;

    state_.prev_gamma_error = gamma_error;
    state_.prev_theta_error = theta_error;
    state_.prev_z_error = z_error;

    steering_i = std::clamp(steering_i, -8.f, 8.f);
    steering_o = std::clamp(steering_o, -8.f, 8.f);
    steering_z = std::clamp(steering_z, -8.f, 8.f);

    // std::cout << leg.GetAngleI() << " " << leg.GetAngleO() << " " <<
    // std::cout << "z: " << az << " st z: " << state_.z_setpoint
    //           << " ctr z: " << leg.GetMotorZ()->GetDirection() * steering_z
    //           << std::endl;

    std::cout << "i: " << leg.GetAngleI() << " ctr i: "
              << steering_i
              //           << " ctr z: " << leg.GetMotorZ()->GetDirection() *
              //           steering_z
              << std::endl;

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

template<class T, class G, class Z> 
void InitializationLegControl::DriveTo(T theta, G gamma, Z z) {
  
}

bool InitializationLegControl::Process(Leg& leg, float dt) override {
  if (done_) {
    return false;
  }

  switch (stage_) {
    case InitializationStage::PreInitialization: {
    } break;

    case InitializationStage::DriveToMinGamma: {
      if (DriveTo(Unlocked, Direction::Negative, Unlocked)) {
        if (!VerifyPosition<Gamma>(leg)) {
          return false;
        }
        locked_theta_ = leg.GetTheta();
        locked_z_ = leg.GetAngleZ();
        // leg.ZeroIORevs();
        stage_ = DriveToSafeZ;
      }
    } break;

    case InitializationStage::DriveToMinZ: {
      if (DriveTo(locked_theta_, leg.GetMinGamma(), Direction::Negative)) {
        measured_z_angle[current_theta_rev_] = leg.GetAngleZ();
        ++current_theta_rev_;
        if (current_theta_rev < 9) {
          stage_ = DriveToSafeZ;
        } else {
          stage_ = PreInit;
          SetActive(false);
        };
        // if (!VerifyPosition<Z>(leg)) {

        //   DriveTo(locked_theta_, leg.GetMinGamma(), Direction::Negative) {

        //   }
        // }
      }
    } break;

    case InitializationStage::DriveToSafeZ: {
      if (DriveTo(locked_theta_, leg.GetMinGamma(), leg.GetInitSafeZ())) {
        stage_ = DriveToNextThetaRev;
        // if (!VerifyPosition<Z>(leg)) {

        //   DriveTo(locked_theta_, leg.GetMinGamma(), Direction::Negative) {

        //   }
        // }
      }
    } break;
    case InitializationStage::DriveToNextThetaRev: {
      if (DriveTo(locked_theta_ + current_theta_rev * 2 * M_PI,
                  leg.GetMinGamma(), leg.GetInitSafeZ())) {
        stage_ = DriveToMinZ;
      }
    } break;
  }