#pragma once

#include <math.h>

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