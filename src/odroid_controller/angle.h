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



inline double DirectionDistance(double from, double to) {
  from = NormalizeAngle(from);
  to = NormalizeAngle(to);
  double diff = std::fabs(to - from);
  return std::min(diff, 2 * M_PI - diff);
}

inline float ClosestAngle(float from, float to) {
  float ret = to - from;

  // if (std::fabs(ret) > 2.0*M_PI - std::fabs(ret)) {
  //   if (to > from) {
  //     ret = to - (from + 2.0 * M_PI);
  //   } else {
  //     ret = to - (from - 2.0 * M_PI);
  //   }
  // }
  
  while (ret > M_PI) {
    ret = ret - 2 * M_PI;
  }

  while (ret < -M_PI) {
    ret = ret + 2 * M_PI;
  }

  return ret;
}