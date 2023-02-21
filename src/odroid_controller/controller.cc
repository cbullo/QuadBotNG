#include "controller.h"

#include <iostream>

Controller::Controller(const Legs& legs,
                       const std::vector<BLDCDriverBoard*>& controllers,
                       std::shared_ptr<StoppedBehavior> stopped_behavior,
                       std::shared_ptr<EStoppedBehavior> estopped_behavior)
    // const Legs& legs, const std::shared_ptr<StoppedBehavior>&
    // stopped_behavior, const std::shared_ptr<StoppedBehavior>&
    // estopped_behavior)
    : legs_(legs), controllers_(controllers) {
  stopped_behavior_ = stopped_behavior;
  estopped_behavior_ = estopped_behavior;

  SubscribeToEvent(stopped_behavior_.get(), EventId::kControlEventStart);
  SubscribeToEvent(stopped_behavior_.get(), EventId::kControlEventNextMode);
  SubscribeToEvent(stopped_behavior_.get(), EventId::kControlEventPreviousMode);

  SubscribeToEvent(estopped_behavior_.get(),
                   EventId::kControlEventReleaseEStop);

  nodes_.push_back(stopped_behavior);
  nodes_.push_back(estopped_behavior);
}

void Controller::DistributeEvents(const std::deque<ControlEvent>& events) {
  for (const auto& e : events) {
    std::cout << "DE2" << std::endl;
    auto values_range = subscriptions_.equal_range(e.event_id);
    for (auto v_it = values_range.first; v_it != values_range.second; ++v_it) {
      std::cout << "DE3" << std::endl;
      if (v_it->second->IsActive()) {
        std::cout << "DE4" << std::endl;
        v_it->second->AddToInputQueue(e);
      }
    }
  }
}

void Controller::Update(float dt) {
  bool trigger_estop = false;
  bool trigger_stop = false;

  for (const auto& node : tickables_) {
    node->Tick(dt);
  }

  for (const auto& node : nodes_) {
    std::deque<ControlEvent> events;
    node->FetchOutputs(events);

    trigger_estop =
        std::find_if(events.begin(), events.end(), [](const auto& e) {
          return e.event_id == EventId::kControlEventEStop;
        }) != events.end();

    if (trigger_estop) {
      std::cout << "Should trigger ESTOP" << std::endl;
      break;
    }

    trigger_stop =
        std::find_if(events.begin(), events.end(), [](const auto& e) {
          return e.event_id == EventId::kControlEventStop;
        }) != events.end();

    DistributeEvents(events);
  }

  if (!trigger_estop) {
    if (trigger_stop) {
      EventNode* active_node = nullptr;
      for (const auto& node : nodes_) {
        if (node->IsActive()) {
          if (!node->IsAlwaysActive()) {
            active_node = node.get();
            node->Deactivate();
          }
        }
      }
      auto* active_behavior = dynamic_cast<Behavior*>(active_node);
      stopped_behavior_->SetPreviousBehavior(active_behavior);
      stopped_behavior_->Activate();
    } else {
      for (const auto& node : nodes_) {
        if (node->IsActive()) {
          node->ProcessInputs();
        }
      }
    }
  } else {
    auto& controllers = GetControllers();
    for (auto& controller : controllers) {
      controller->SetErrorState(true);
    }
    estopped_behavior_->Activate();
  }

  if (legs_.fl) {
    legs_.fl->UpdateControl(dt);
  }
  if (legs_.fr) {
    legs_.fr->UpdateControl(dt);
  }
  if (legs_.bl) {
    legs_.bl->UpdateControl(dt);
  }
  if (legs_.br) {
    legs_.br->UpdateControl(dt);
  }
  auto& controllers = GetControllers();
  for (auto* controller : controllers) {
    controller->Tick();
  }
}

// void Controller::SetBehavior(Behavior* b) {
//   current_behavior_ = b;
//   current_behavior_->Start(legs_);
// };

// void Controller::SetMode(ControllerMode new_mode) {
//   switch (new_mode) {
//     case ControllerMode::kThetaGammaDrive:
//       SetBehavior(&theta_gamma_behavior_);
//       break;
//     case ControllerMode::kXYZDrive:
//       SetBehavior(&xyz_behavior_);
//       break;
//     case ControllerMode::kWalk:
//       // TODO: Waiting for all legs :)
//       break;
//   }
// }

void Behavior::Activate() {
  EventNode::Activate();
  for (auto& cb : activation_callbacks_) {
    cb(*this);
  }
}
void Behavior::Deactivate() {
  EventNode::Deactivate();
  for (auto& cb : deactivation_callbacks_) {
    cb(*this);
  }
}

void Behavior::ProcessInputEvents(const std::deque<ControlEvent>& events) {
  // for (const auto& event : events) {
  //   switch (event.event_id) {
  //     case EventId::kControlEventNextMode: {
  //       if (NextBehavior()) {
  //         this->Deactivate();
  //         NextBehavior()->Activate();
  //       }
  //     } break;
  //     case EventId::kControlEventPreviousMode: {
  //       if (PreviousBehavior()) {
  //         this->Deactivate();
  //         PreviousBehavior()->Activate();
  //       }
  //     } break;
  //     default:
  //       break;
  //   }
  // }
}

void StoppedBehavior::ProcessInputEvents(
    const std::deque<ControlEvent>& events) {
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventStart:
        std::cout << "trying activate previous" << std::endl;
        if (previous_behavior_) {
          Deactivate();
          std::cout << "Activate previous" << std::endl;
          previous_behavior_->Activate();
        }
        break;
      case EventId::kControlEventNextMode:
        if (previous_behavior_ && previous_behavior_->NextBehavior()) {
          previous_behavior_ = previous_behavior_->NextBehavior();
        }
        break;
      case EventId::kControlEventPreviousMode:
        if (previous_behavior_ && previous_behavior_->PreviousBehavior()) {
          previous_behavior_ = previous_behavior_->PreviousBehavior();
        }
        break;
      default:
        break;
    }
  }
}

void EStoppedBehavior::ProcessInputEvents(
    const std::deque<ControlEvent>& events) {
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventReleaseEStop: {
        Deactivate();
        auto& controllers = GetController()->GetControllers();
        for (auto& controller : controllers) {
          controller->SetErrorState(false);
        }
        // for (int i = 0; i < 4; ++i) {
        //   auto* leg = legs.GetLeg(i);
        //   if (!leg) {
        //     continue;
        //   }
        //   leg->GetMotorI()->GetController()->SetErrorState(false);
        //   leg->GetMotorO()->GetController()->SetErrorState(false);
        //   leg->GetMotorZ()->GetController()->SetErrorState(false);
        // }
      } break;
      default:
        break;
    }
  }
}

void LegTestingBehavior::SendSetupData(Leg& leg) {
  leg.GetMotorI()->GetController()->SendSetCommand(0, CMD_STATE,
                                                   static_cast<uint8_t>(3));
  leg.GetMotorO()->GetController()->SendSetCommand(0, CMD_STATE,
                                                   static_cast<uint8_t>(3));
  leg.GetMotorZ()->GetController()->SendSetCommand(0, CMD_STATE,
                                                   static_cast<uint8_t>(3));
}

void LegTestingBehavior::EnableControlForCurrentLeg(Legs& legs) {
  std::cout << "Enable control for leg " << selected_leg_ << std::endl;
  auto* leg = legs.GetLeg(selected_leg_);
  if (leg) {
    leg->SetControl(&leg_control_);
  }
}

void LegTestingBehavior::DisableControlForCurrentLeg(Legs& legs) {
  auto* leg = legs.GetLeg(selected_leg_);
  if (leg) {
    leg->SetControl(nullptr);
  }
}

// void LegTestingBehavior::Start(Legs& legs) {
//   EnableControlForCurrentLeg(legs);
// };

// void LegTestingBehavior::Stop(Legs& legs) {
//   DisableControlForCurrentLeg(legs);
// };

void LegTestingBehavior::ProcessInputEvents(
    const std::deque<ControlEvent>& events) {
  Behavior::ProcessInputEvents(events);
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventNextItem: {
        while (true) {
          selected_leg_ = (selected_leg_ + 1) % 4;
          if (GetController()->GetLegs().GetLeg(selected_leg_)) {
            break;
          }
        }
      } break;
      case EventId::kControlEventPreviousItem: {
        while (true) {
          selected_leg_ = (selected_leg_ + 3) % 4;
          if (GetController()->GetLegs().GetLeg(selected_leg_)) {
            break;
          }
        }
      } break;
      case EventId::kControlEventConfirm: {
        EnableControlForCurrentLeg(GetController()->GetLegs());
        SendSetupData(*GetController()->GetLegs().GetLeg(selected_leg_));
      }
      case EventId::kControlEventLegTheta: {
        leg_control_.SetGammaSetpoint(event.value);
      } break;
      case EventId::kControlEventLegGamma: {
        leg_control_.SetThetaSetpoint(event.value);
      } break;
      case EventId::kControlEventLegTilt: {
        leg_control_.SetZSetpoint(event.value);
      } break;
    }
  }
};

void OutputMode(CalibrationLegControl::CalibrationMode mode, int motor_index) {
  switch (mode) {
    case CalibrationLegControl::CalibrationMode::ElectricZero: {
      // std::cout << "\e[2KElectric Zero, Motor: " << motor_index;
      std::cout << "Electric Zero, Motor: " << motor_index << std::endl;
      break;
    }
    case CalibrationLegControl::CalibrationMode::LinearizationReading: {
      std::cout << "Linearization, Motor: " << motor_index << std::endl;
      break;
    }
    case CalibrationLegControl::CalibrationMode::LinearizationValidation: {
      std::cout << "Linearization Validation, Motor: " << motor_index
                << std::endl;
      break;
    }
  }
}

void MotorTestingBehavior::Activate() {
  Behavior::Activate();
  OutputMode(leg_control_.GetCalibrationMode(), selected_motor_);
}

void MotorTestingBehavior::EnableControlForCurrentLeg(Legs& legs) {
  int selected_leg = selected_motor_ / 3;
  auto* leg = legs.GetLeg(selected_leg);
  if (leg) {
    leg->SetControl(&leg_control_);
    leg_control_.SetMotorIndex(selected_motor_ % 4);
  }
}

void MotorTestingBehavior::DisableControlForCurrentLeg(Legs& legs) {
  int selected_leg = selected_motor_ / 3;
  auto* leg = legs.GetLeg(selected_leg);
  if (leg) {
    leg->SetControl(nullptr);
  }
}

void MotorTestingBehavior::ProcessInputEvents(
    const std::deque<ControlEvent>& events) {
  Behavior::ProcessInputEvents(events);
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventNextItem: {
        selected_motor_ = (selected_motor_ + 1) % 12;
        OutputMode(leg_control_.GetCalibrationMode(), selected_motor_);
        // std::cout << selected_motor_ << std::endl;
        // DisableControlForCurrentLeg(GetController()->GetLegs());
        // selected_leg_ = selected_motor_ / 3;
        // EnableControlForCurrentLeg(GetController()->GetLegs());
        // leg_control_.SetMotorIndex(selected_motor_ % 3);

        // if (selected_motor_ % 3 == 0) {
        //   DisableControlForCurrentLeg(GetController()->GetLegs());
        //   while (true) {
        //     selected_leg_ = (selected_leg_ + 1) % 4;
        //     if (GetController()->GetLegs().GetLeg(selected_leg_)) {
        //       break;
        //     }
        //     selected_motor_ = (selected_motor_ + 3) % 12;
        //   }
        //   EnableControlForCurrentLeg(GetController()->GetLegs());
        //   leg_control_.SetMotorIndex(selected_motor_ % 4);
        // }
      } break;
      case EventId::kControlEventPreviousItem: {
        selected_motor_ = (selected_motor_ + 11) % 12;
        OutputMode(leg_control_.GetCalibrationMode(), selected_motor_);
        // std::cout << selected_motor_ << std::endl;
        // DisableControlForCurrentLeg(GetController()->GetLegs());
        // selected_leg_ = selected_motor_ / 3;
        // EnableControlForCurrentLeg(GetController()->GetLegs());
        // leg_control_.SetMotorIndex(selected_motor_ % 3);

        // if (selected_motor_ % 3 == 0) {
        //   DisableControlForCurrentLeg(GetController()->GetLegs());
        //   while (true) {
        //     selected_leg_ = (selected_leg_ + 1) % 4;
        //     if (GetController()->GetLegs().GetLeg(selected_leg_)) {
        //       break;
        //     }
        //     selected_motor_ = (selected_motor_ + 3) % 12;
        //   }
        //   EnableControlForCurrentLeg(GetController()->GetLegs());
        //   leg_control_.SetMotorIndex(selected_motor_ % 4);
        // }
      } break;
      case EventId::kControlEventNextMode: {
        std::cout << "Next mode" << std::endl;
        auto mode = leg_control_.GetCalibrationMode();
        switch (mode) {
          case CalibrationLegControl::CalibrationMode::LinearizationReading:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::
                    LinearizationValidation);
            break;
          case CalibrationLegControl::CalibrationMode::LinearizationValidation:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::ElectricZero);
            break;
          case CalibrationLegControl::CalibrationMode::ElectricZero:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::LinearizationReading);
            break;
        }
      } break;
      case EventId::kControlEventPreviousMode: {
        std::cout << "Previous mode" << std::endl;
        auto mode = leg_control_.GetCalibrationMode();
        switch (mode) {
          case CalibrationLegControl::CalibrationMode::LinearizationReading:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::ElectricZero);
            break;
          case CalibrationLegControl::CalibrationMode::LinearizationValidation:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::LinearizationReading);
            break;
          case CalibrationLegControl::CalibrationMode::ElectricZero:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::
                    LinearizationValidation);
            break;
        }
      } break;
      case EventId::kControlEventConfirm: {
        if (!leg_control_.IsRunning()) {
          DisableControlForCurrentLeg(GetController()->GetLegs());
          EnableControlForCurrentLeg(GetController()->GetLegs());
          std::cout << "Confirm - SendSetupData " << selected_motor_
                    << std::endl;
          int selected_leg = selected_motor_ / 3;
          auto* leg = GetController()->GetLegs().GetLeg(selected_leg);
          if (!leg) {
            std::cout << "No leg " << selected_leg;
            break;
          }
          leg_control_.SendSetupData(*leg);
          leg_control_.SetRunning(true);
          // std::cout << "\n";
        } else {
          // std::cout << "Confirm - running" << std::endl;
          switch (leg_control_.GetCalibrationMode()) {
            case CalibrationLegControl::CalibrationMode::LinearizationReading: {
              auto next_step = leg_control_.GetStep() + 1;
              sensor_values_[leg_control_.GetStep()] = leg_control_.raw_angle_;
              if (next_step < 64) {
                leg_control_.SetStep(next_step);
              } else {
                leg_control_.SetStep(0);
                leg_control_.SetRunning(false);
                for (int i = 0; i < 64; ++i) {
                  std::cout << i << "," << sensor_values_[i] << std::endl;
                }
              }
              std::cout << std::endl;
            } break;

            case CalibrationLegControl::CalibrationMode::ElectricZero:
            case CalibrationLegControl::CalibrationMode::
                LinearizationValidation:
              break;
          }
        }
      } break;
      default:
        break;
    }
  }
};

bool ThetaGammaZLegControl::Process(Leg& leg, float dt) {
  delay_ -= dt;
  if (delay_ <= 0.f) {
    double az = leg.GetAngleZ();

    double theta = leg.GetTheta();
    double gamma = leg.GetGamma();

    double theta_error = ClosestAngle(theta, state_.theta_setpoint);
    state_.i_theta_sum =
        state_.i_theta_sum + dt * 0.5 * (theta_error + state_.prev_theta_error);
    double p_term_theta = leg.theta_pd_config.p * (theta_error);
    double d_term_theta =
        -leg.theta_pd_config.d * (theta_error - state_.prev_theta_error) / dt;
    double i_term_theta = leg.theta_pd_config.i * state_.i_theta_sum;

    double gamma_error = ClosestAngle(gamma, state_.gamma_setpoint);
    state_.i_gamma_sum =
        state_.i_gamma_sum + dt * 0.5 * (gamma_error + state_.prev_gamma_error);

    double p_term_gamma = leg.gamma_pd_config.p * (gamma_error);
    double d_term_gamma =
        -leg.gamma_pd_config.d * (gamma_error - state_.prev_gamma_error) / dt;
    double i_term_gamma = leg.gamma_pd_config.i * state_.i_gamma_sum;

    double tau_theta = p_term_theta + d_term_theta + i_term_theta;
    double tau_gamma = p_term_gamma + d_term_gamma + i_term_gamma;

    double z_error = ClosestAngle(az, state_.z_setpoint);
    state_.i_z_sum =
        state_.i_z_sum + dt * 0.5 * (z_error + state_.prev_z_error);
    double p_term_z = leg.z_pd_config.p * (z_error);
    double d_term_z = -leg.z_pd_config.d * (z_error - state_.prev_z_error) / dt;
    double i_term_z = leg.z_pd_config.i * state_.i_z_sum;
    double tau_z = p_term_z + d_term_z + i_term_z;

    float steering_i =  (0.5 * tau_theta - 0.5 * tau_gamma);
    float steering_o =  (0.5 * tau_theta + 0.5 * tau_gamma);
    float steering_z = tau_z /*- 0.75f * tau_gamma*/;

    state_.prev_gamma_error = gamma_error;
    state_.prev_theta_error = theta_error;
    state_.prev_z_error = z_error;

    steering_i = std::clamp(50.f * steering_i, -5.f, 5.f);
    steering_o = std::clamp(50.f * steering_o, -5.f, 5.f);
    steering_z = std::clamp(50.f * steering_z, -5.f, 5.f);

    // std::cout << leg.GetAngleI() << " " << leg.GetAngleO() << " " <<
    // leg.GetAngleZ() << std::endl;

    // std::cout << theta << " " << state_.theta_setpoint << " " << gamma << " "
    //           << state_.gamma_setpoint << " " << steering_i << " " << steering_o
    //           << std::endl;

    std::cout << theta << " " << gamma << std::endl;

    leg.GetMotorI()->GetController()->SendSetCommand(
        leg.GetMotorI()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(-512 * steering_i));
    leg.GetMotorO()->GetController()->SendSetCommand(
        leg.GetMotorO()->GetIndex(), CMD_MOTOR_VOLTAGE,
        static_cast<int16_t>(-512 * steering_o));
    // leg.GetMotorZ()->GetController()->SendSetCommand(
    //     leg.GetMotorZ()->GetIndex(), CMD_MOTOR_VOLTAGE,
    //     static_cast<int16_t>(512 * steering_z));

    delay_ = 0.01f;
  }

  return true;
}

void CalibrationLegControl::SetCalibrationMode(
    CalibrationLegControl::CalibrationMode mode) {
  std::cout << "Calibration mode set: " << (int)mode << std::endl;
  calibration_mode_ = mode;
  step_ = 0;
  SetRunning(false);
  OutputMode(calibration_mode_, motor_index_);
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
      //                                        static_cast<int16_t>(512 * 5));

      motor->GetController()->SendSetCommand(0, CMD_STATE,
                                             static_cast<uint8_t>(3));

      step_ = 0;
      break;
  }
}

Motor* CalibrationLegControl::GetMotor(Leg& leg, int index) const {
  switch (index) {
    case 0:
      return leg.GetMotorI();
    case 1:
      return leg.GetMotorO();
    case 2:
      return leg.GetMotorZ();
    default:
      return nullptr;
  }
}

bool CalibrationLegControl::Process(Leg& leg, float dt) {
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
              std::cout << "Zero angle: " << ((angle * 7) % 4096) / 7
                        << std::endl;
              step_ += 4;
            }
            break;
          }
        }

        if (next_read_delta_ <= 0.f) {
          motor->GetController()->SendSetCommand(
              motor->GetIndex(), CMD_MOTOR_PHASE,
              static_cast<int16_t>(step_ % 4096),
              static_cast<int16_t>(2 * 512));
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
            next_read_delta_ = 0.5f;
            motor->GetController()->SendSetCommand(
                motor->GetIndex(), CMD_MOTOR_PHASE,
                static_cast<int16_t>(step_ % 4096),
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
              std::cout << "Set: " << set_angle << " Measured: " << measured
                        << " Error: " << set_angle - measured << std::endl;
              step_ += 8;
            }
            break;
          }
        }

        if (next_read_delta_ <= 0.f) {
          motor->GetController()->SendSetCommand(
              motor->GetIndex(), CMD_MOTOR_PHASE,
              static_cast<int16_t>(step_ % 4096),
              static_cast<int16_t>(2 * 512));
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
  }

  return true;
}
