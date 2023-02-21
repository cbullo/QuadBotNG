#pragma once

class Leg;

class LegControl {
 public:
  virtual bool Process(Leg& leg, float dt) = 0;
};
