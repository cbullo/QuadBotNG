#pragma once

enum class EventId {
  kControlEventForward = 1,
  kControlEventSide = 2,
  kControlEventTurn = 3,
  kControlEventTilt = 4,
  kControlEventRoll = 5,
  kControlEventLegTheta = 6,
  kControlEventLegGamma = 7,
  kControlEventNextMode = 8,
  kControlEventPreviousMode = 9,
  kControlEventNextItem = 10,
  kControlEventPreviousItem = 11,
  kControlEventEStop = 12,
  kControlEventReleaseEStop = 13,
  kControlEventStop = 14,
  kControlEventStart = 15,
  kControlEventLegTilt = 16,
  kControlEventConfirm = 17,
  kInitializationDone = 18
};