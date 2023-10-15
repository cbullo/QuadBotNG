#include "controller.h"

#include <iostream>

#include "log_helper.h"

Controller::Controller(const Legs& legs,
                       const std::vector<ControllerType*>& controllers,
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

  duration_ += dt;

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

  auto& controllers = GetControllers();
  bool any_transmission_bound = false;
  for (auto& controller : controllers) {
    any_transmission_bound = any_transmission_bound || controller->IsTransmissionBound();
  }

  dt_acc_ += dt;
  if (!any_transmission_bound) {
    if (legs_.fl) {
      legs_.fl->UpdateControl(dt_acc_);
    }
    if (legs_.fr) {
      legs_.fr->UpdateControl(dt_acc_);
    }
    if (legs_.bl) {
      legs_.bl->UpdateControl(dt_acc_);
    }
    if (legs_.br) {
      legs_.br->UpdateControl(dt_acc_);
    }
  
    for (auto* controller : controllers) {
      controller->Tick();
    }

    dt_acc_ = 0.f;
  } else {
    std::cout << "Trasmission bound, skipping. Current delta: " << dt_acc_ << std::endl;
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
  if (auto* motor = leg.GetMotorI()) {
    motor->GetController()->SendSetCommand(0, CMD_STATE,
                                           static_cast<uint8_t>(3));
  }
  if (auto* motor = leg.GetMotorO()) {
    motor->GetController()->SendSetCommand(0, CMD_STATE,
                                           static_cast<uint8_t>(3));
  }
  if (auto* motor = leg.GetMotorZ()) {
    motor->GetController()->SendSetCommand(0, CMD_STATE,
                                           static_cast<uint8_t>(3));
  }
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

void OutputLeg(const Legs& legs, int leg_index) {
  std::string leg_name = "<NO_LEG>";
  if (auto* leg = legs.GetLeg(leg_index)) {
    leg_name = leg->GetName();
  }

  std::cout << "Leg: " << leg_name << std::endl;
}

void LegTestingBehavior::Activate() {
  Behavior::Activate();
  OutputLeg(GetController()->GetLegs(), selected_leg_);
}

void LegTestingBehavior::ProcessInputEvents(
    const std::deque<ControlEvent>& events) {
  Behavior::ProcessInputEvents(events);
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventNextItem: {
        while (true) {
          selected_leg_ = (selected_leg_ + 1) % 4;
          if (GetController()->GetLegs().GetLeg(selected_leg_)) {
            OutputLeg(GetController()->GetLegs(), selected_leg_);
            break;
          }
        }
      } break;
      case EventId::kControlEventPreviousItem: {
        while (true) {
          selected_leg_ = (selected_leg_ + 3) % 4;
          if (GetController()->GetLegs().GetLeg(selected_leg_)) {
            OutputLeg(GetController()->GetLegs(), selected_leg_);
            break;
          }
        }
      } break;
      case EventId::kControlEventConfirm: {
        EnableControlForCurrentLeg(GetController()->GetLegs());
        SendSetupData(*GetController()->GetLegs().GetLeg(selected_leg_));
      } break;
      case EventId::kControlEventLegTheta: {
        leg_control_.SetThetaSetpoint(event.value);
      } break;
      case EventId::kControlEventLegGamma: {
        leg_control_.SetGammaSetpoint(event.value);
      } break;
      case EventId::kControlEventLegTilt: {
        leg_control_.SetZSetpoint(event.value);
      } break;
    }
  }
};

void OutputMode(CalibrationLegControl::CalibrationMode mode, const Legs& legs,
                int motor_index) {
  int selected_leg = motor_index / 3;

  std::string leg_name = "<NO_LEG>";
  std::string motor_name = "<NO_MOTOR>";
  std::string controller_name = "<NO_CONTROLLER>";
  std::string motor_index_str = "<?>";
  if (auto* leg = legs.GetLeg(selected_leg)) {
    leg_name = leg->GetName();
    auto* motor = GetMotor(*leg, motor_index);
    if (motor) {
      motor_name = motor->GetName();
      motor_index_str = std::to_string(motor->GetIndex());
      auto* controller = motor->GetController();
      if (controller) {
        controller_name = controller->GetName();
      }
    }
  }

  switch (mode) {
    case CalibrationLegControl::CalibrationMode::ElectricZero: {
      // std::cout << "\e[2KElectric Zero, Motor: " << motor_index;
      std::cout << "Electric Zero";
      break;
    }
    case CalibrationLegControl::CalibrationMode::LinearizationReading: {
      std::cout << "Linearization";
      break;
    }
    case CalibrationLegControl::CalibrationMode::LinearizationValidation: {
      std::cout << "Linearization Validation";
      break;
    }
    case CalibrationLegControl::CalibrationMode::MotorVoltage: {
      std::cout << "Voltage Control";
      break;
    }
  }

  std::cout << ", Motor: " << motor_name << "(" << leg_name << ", "
            << controller_name << ", " << motor_index_str << ")" << std::endl;
}

void MotorTestingBehavior::Activate() {
  Behavior::Activate();
  OutputMode(leg_control_.GetCalibrationMode(), GetController()->GetLegs(),
             selected_motor_);
}

void MotorTestingBehavior::EnableControlForCurrentLeg(Legs& legs) {
  int selected_leg = selected_motor_ / 3;
  auto* leg = legs.GetLeg(selected_leg);
  if (leg) {
    leg->SetControl(&leg_control_);
    leg_control_.SetMotorIndex(selected_motor_ % 3);
  }
}

void MotorTestingBehavior::DisableControlForCurrentLeg(Legs& legs) {
  int selected_leg = selected_motor_ / 3;
  auto* leg = legs.GetLeg(selected_leg);
  if (leg) {
    if (auto* m = leg->GetMotorI()) {
      auto* c = m->GetController();
      c->SendSetCommand(0, CMD_STATE, static_cast<uint8_t>(2));
    }
  }
}

void MotorTestingBehavior::ProcessInputEvents(
    const std::deque<ControlEvent>& events) {
  Behavior::ProcessInputEvents(events);
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventLegTilt: {
        leg_control_.SetMotorControl(event.value);
      } break;
      case EventId::kControlEventNextItem: {
        selected_motor_ = (selected_motor_ + 1) % 12;
        OutputMode(leg_control_.GetCalibrationMode(),
                   GetController()->GetLegs(), selected_motor_);
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
        //   leg_control_.SetMotorIndex(selected_motor_ % 3);
        // }
      } break;
      case EventId::kControlEventPreviousItem: {
        selected_motor_ = (selected_motor_ + 11) % 12;

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
        //   leg_control_.SetMotorIndex(selected_motor_ % );
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
                CalibrationLegControl::CalibrationMode::MotorVoltage);
            break;
          case CalibrationLegControl::CalibrationMode::MotorVoltage:
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
                CalibrationLegControl::CalibrationMode::MotorVoltage);
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
          case CalibrationLegControl::CalibrationMode::MotorVoltage:
            leg_control_.SetCalibrationMode(
                CalibrationLegControl::CalibrationMode::ElectricZero);
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
            case CalibrationLegControl::CalibrationMode::MotorVoltage:
              DisableControlForCurrentLeg(GetController()->GetLegs());
              leg_control_.SetRunning(false);
              break;
          }
        }
      } break;
      default:
        break;
    }
    OutputMode(leg_control_.GetCalibrationMode(), GetController()->GetLegs(),
               selected_motor_);
  }
};

void WalkBehavior::Activate() {
  std::cout << "Walk behavior" << std::endl;
  Behavior::Activate();
}

void WalkBehavior::ProcessInputEvents(const std::deque<ControlEvent>& events) {
  Behavior::ProcessInputEvents(events);
  const auto& legs = GetController()->GetLegs();
  for (const auto& event : events) {
    switch (event.event_id) {
      case EventId::kControlEventConfirm: {
        auto leg = legs.bl;
        leg->SetControl(init_control_);
        init_control_->Restart();
        if (auto* motor = leg->GetMotorI()) {
          motor->GetController()->SendSetCommand(0, CMD_STATE,
                                                 static_cast<uint8_t>(3));
        }
        if (auto* motor = leg->GetMotorO()) {
          motor->GetController()->SendSetCommand(0, CMD_STATE,
                                                 static_cast<uint8_t>(3));
        }
        if (auto* motor = leg->GetMotorZ()) {
          motor->GetController()->SendSetCommand(0, CMD_STATE,
                                                 static_cast<uint8_t>(3));
        }
      } break;
    }
  }
}
