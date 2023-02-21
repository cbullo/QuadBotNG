#pragma once

#include <deque>
#include <iostream>

#include "event_ids.h"

struct ControlEvent {
  EventId event_id;
  float value;
};

class EventNode {
  friend class Controller;

 public:
  virtual ~EventNode(){};
  virtual void Activate() { active_ = true; }
  virtual void Deactivate() {
    if (!always_active_) {
      active_ = false;
    };
  }
  bool IsActive() { return active_ || always_active_; }
  bool IsAlwaysActive() { return always_active_; };

  void SubscribeToEvent(EventId event);
  virtual void ProcessInputEvents(const std::deque<ControlEvent>& events) = 0;

  virtual void Tick(float dt){};

 protected:
  void OutputEvent(const ControlEvent& event) {
    output_queue_.push_back(event);
  }

  bool always_active_ = false;

 private:
  void FetchOutputs(std::deque<ControlEvent>& out_events) {
    if (output_queue_.size() > 0) {
      std::cout << "Some events fetch" << std::endl;
    }
    output_queue_.swap(out_events);
  }
  void AddToInputQueue(ControlEvent event) { input_queue_.push_back(event); }
  void ProcessInputs() {
    if (input_queue_.size() > 0) {
      std::cout << "Some events process" << std::endl;
    }
    ProcessInputEvents(input_queue_);
    input_queue_.clear();
  }

  std::deque<ControlEvent> input_queue_;
  std::deque<ControlEvent> output_queue_;
  bool active_ = false;
};