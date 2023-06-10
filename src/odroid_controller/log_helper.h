#pragma once

#include "leg.h"
#include "motor.h"

inline Motor* GetMotor(Leg& leg, int index) {
  switch (index % 3) {
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
