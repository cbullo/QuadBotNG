#pragma once

#include <math.h>

inline float NormalizeAngle(double ret) {
  while (ret >= 2 * M_PI) {
    ret = ret - 2 * M_PI;
  }

  while (ret < -2.f * M_PI) {
    ret = ret + 2 * M_PI;
  }

  return ret;
}

inline double ClosestAngle(double from, double to) {
  double ret = (double)to - (double)from;
  while (ret > M_PI) {
    ret = ret - 2 * M_PI;
  }

  while (ret < -M_PI) {
    ret = ret + 2 * M_PI;
  }

  return ret;
}