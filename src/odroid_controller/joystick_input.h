#pragma once

// #include <arpa/inet.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <queue>
#include <string>
#include <vector>

#include "event_factory.h"
#include "event_ids.h"

enum class JoystickButtonIds {
  SelectButton = 0x0,
  StartButton = 0x3,
  HatRight = 0x5,
  HatLeft = 0x7,
  HatUp = 0x4,
  HatDown = 0x6,
  Cross = 0xE
};

enum class JoystickAxisIds {
  LeftStickHorz = 0,
  LeftStickVert = 1,
  RightStickHorz = 2,
  RightStickVert = 3
};

struct JoystickState {
  bool buttons[255] = {false};
  float axes[255] = {0.f};
};

class JoystickControlScheme {
 public:
  virtual void ProcessInput(const JoystickState& state,
                            const js_event& joystick_event,
                            std::deque<ControlEvent>& events) = 0;
};

// class ThetaGammaScheme {
//   public:
//     virtual void
// };

class EStoppedScheme : public JoystickControlScheme {
  void ProcessInput(const JoystickState& state, const js_event& joystick_event,
                    std::deque<ControlEvent>& events) override {
    if (joystick_event.number ==
        static_cast<uint8_t>(JoystickButtonIds::SelectButton)) {
      select_pressed_ = joystick_event.value;
    } else if (joystick_event.number ==
               static_cast<uint8_t>(JoystickButtonIds::StartButton)) {
      start_pressed_ = joystick_event.value;
    }

    if (start_pressed_ && select_pressed_) {
      events.push_back({EventId::kControlEventReleaseEStop, 0.f});
    }
  }

 private:
  bool select_pressed_ = false;
  bool start_pressed_ = false;
};

bool IsAxisEvent(const js_event& event, JoystickAxisIds id) {
  return (event.type == JS_EVENT_AXIS &&
          event.number == static_cast<uint8_t>(id));
}

float GetAxisValue(const JoystickState& state, const js_event& event,
                   JoystickAxisIds id) {
  if (IsAxisEvent(event, id)) {
    return event.value;
  } else {
    return state.axes[static_cast<uint8_t>(id)];
  }
}

bool Pressed(const JoystickState& state, const js_event& event,
             JoystickButtonIds id) {
  return (event.type == JS_EVENT_BUTTON &&
          event.number == static_cast<uint8_t>(id) && event.value != 0 &&
          !state.buttons[static_cast<uint8_t>(id)]);
}

bool Released(const JoystickState& state, const js_event& event,
              JoystickButtonIds id) {
  return (event.type == JS_EVENT_BUTTON &&
          event.number == static_cast<uint8_t>(id) && event.value == 0 &&
          state.buttons[static_cast<uint8_t>(id)]);
}

bool HandleCommonInputs(const JoystickState& state,
                        const js_event& joystick_event,
                        std::deque<ControlEvent>& events) {
  if (Pressed(state, joystick_event, JoystickButtonIds::StartButton)) {
    events.push_back({EventId::kControlEventStop, 0.f});
    return true;
  }

  return false;
}

class StoppedScheme : public JoystickControlScheme {
  void ProcessInput(const JoystickState& state, const js_event& joystick_event,
                    std::deque<ControlEvent>& events) override {
    // if (HandleCommonInputs(state, joystick_event, events)) {
    //   return;
    // }

    if (Pressed(state, joystick_event, JoystickButtonIds::StartButton)) {
      std::cout << "Should start" << std::endl;
      events.push_back({EventId::kControlEventStart, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::HatUp)) {
      events.push_back({EventId::kControlEventNextMode, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::HatDown)) {
      events.push_back({EventId::kControlEventPreviousMode, 0.f});
    }
  }
};

class ThetaGammaScheme : public JoystickControlScheme {
  void ProcessInput(const JoystickState& state, const js_event& joystick_event,
                    std::deque<ControlEvent>& events) {
    if (HandleCommonInputs(state, joystick_event, events)) {
      return;
    }

    if (Pressed(state, joystick_event, JoystickButtonIds::HatRight)) {
      events.push_back({EventId::kControlEventNextItem, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::HatLeft)) {
      events.push_back({EventId::kControlEventPreviousItem, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::Cross)) {
      events.push_back({EventId::kControlEventConfirm, 0.f});
    }

    if (IsAxisEvent(joystick_event, JoystickAxisIds::LeftStickHorz) ||
        IsAxisEvent(joystick_event, JoystickAxisIds::LeftStickVert) ||
        IsAxisEvent(joystick_event, JoystickAxisIds::RightStickHorz) ||
        IsAxisEvent(joystick_event, JoystickAxisIds::RightStickVert)) {
      float x =
          GetAxisValue(state, joystick_event, JoystickAxisIds::LeftStickHorz);
      float y =
          GetAxisValue(state, joystick_event, JoystickAxisIds::LeftStickVert);
      float z =
          GetAxisValue(state, joystick_event, JoystickAxisIds::RightStickHorz);

      float l =
          GetAxisValue(state, joystick_event, JoystickAxisIds::RightStickVert);

      x /= 32767.f;
      y /= 32767.f;
      z /= 32767.f;
      l /= 32767.f;

      float length = sqrtf(x * x + y * y);
      if (length > 0.1f) {
        // if (length > 1.f) {
        //   length = 1.f;
        // }

        float angle = atan2f(-y, x);
        events.push_back({EventId::kControlEventLegTheta, angle});
        // events.push_back({EventId::kControlEventLegGamma, 0.5f * length});
      }
      if (fabs(z) > 0.1f) {
        events.push_back({EventId::kControlEventLegTilt, z});
      }
      if (fabs(l) > 0.05f) {
        events.push_back({EventId::kControlEventLegGamma, l});
      }
    }
  }
};

class CalibrationScheme : public JoystickControlScheme {
  void ProcessInput(const JoystickState& state, const js_event& joystick_event,
                    std::deque<ControlEvent>& events) override {
    // std::cout << (int)joystick_event.type << " " <<
    // (int)joystick_event.number
    //           << " " << (int)joystick_event.value << std::endl;
    // std::cout << "Calibration scheme 1" << std::endl;
    if (HandleCommonInputs(state, joystick_event, events)) {
      return;
    }
    // std::cout << "Calibration scheme 2" << std::endl;

    if (Pressed(state, joystick_event, JoystickButtonIds::HatDown)) {
      events.push_back({EventId::kControlEventNextMode, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::HatUp)) {
      events.push_back({EventId::kControlEventPreviousMode, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::HatRight)) {
      events.push_back({EventId::kControlEventNextItem, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::HatLeft)) {
      events.push_back({EventId::kControlEventPreviousItem, 0.f});
    } else if (Pressed(state, joystick_event, JoystickButtonIds::Cross)) {
      events.push_back({EventId::kControlEventConfirm, 0.f});
    }

    if (IsAxisEvent(joystick_event, JoystickAxisIds::RightStickHorz)) {
      float z =
          GetAxisValue(state, joystick_event, JoystickAxisIds::RightStickHorz);

      z /= 32767.f;
      events.push_back({EventId::kControlEventLegTilt, z});
    }

    // std::cout << "Calibration scheme 3" << std::endl;
  }
};

class JoystickInput : public EventNode {
 public:
  JoystickInput(const std::string& device_address)
      : device_address_(device_address) {
    always_active_ = true;
  };
  void Init() {
    if (joystick_fd_ == -1) {
      joystick_fd_ = open(device_address_.c_str(), O_RDONLY | O_NONBLOCK);
    }
  }

  void Tick(float dt) override { ReadInput(); };

  void ProcessInputEvents(const std::deque<ControlEvent>& events) override {
    // for (const auto& e : events) {
    //   switch (e.event_id) {
    //     case EventId::kControlEventEStop:
    //       scheme_ = &estopped_scheme_;
    //       break;
    //     case EventId::kControlEventStop:
    //       scheme_ = &stopped_scheme_;
    //       break;
    //     default:
    //       break;
    //   };
    // }
    // ReadInput();
  }

  void ReadInput() {
    bool trigger_estop = false;
    bool trigger_stop = false;
    if (joystick_fd_ == -1) {
      trigger_estop = true;
    }

    std::deque<ControlEvent> events;
    while (joystick_fd_ >= 0) {
      int bytes;
      js_event event;
      bytes = read(joystick_fd_, &event, sizeof(event));

      if (bytes == -1) {
        if (errno != EAGAIN) {
          trigger_estop = true;
          close(joystick_fd_);
          joystick_fd_ = -1;
        }
        break;
      }

      if (event.type == JS_EVENT_INIT) {
      } else if (event.type == JS_EVENT_BUTTON) {
        // std::cout << (int)event.number << std::endl;
        if (event.number == static_cast<int>(JoystickButtonIds::SelectButton) &&
            event.value == 1) {
          trigger_estop = true;
          break;
        } else {
          // std::cout << "Scheme: " << scheme_ << std::endl;
          scheme_->ProcessInput(current_state_, event, events);
          // std::cout << "Post process input: " << (int)event.number <<
          // std::endl;
          current_state_.buttons[event.number] = event.value;
        }
      } else if (event.type == JS_EVENT_AXIS) {
        scheme_->ProcessInput(current_state_, event, events);
        current_state_.axes[event.number] = event.value;
      }
    }

    if (trigger_estop) {
      std::cout << "ReadInput 5" << std::endl;
      OutputEvent({EventId::kControlEventEStop, 0.f});
    } else {
      for (const auto& e : events) {
        std::cout << "Output event" << std::endl;
        OutputEvent(e);
      }
    }
  }

  void SetScheme(std::shared_ptr<JoystickControlScheme> scheme) {
    scheme_ = scheme;
  }

 private:
  std::string device_address_;
  int joystick_fd_ = -1;

  JoystickState current_state_;
  std::shared_ptr<JoystickControlScheme> scheme_ = {};
};