#include "Arduino.h"

template <uint8_t kRangePower, uint8_t kSegmentsPower>
class PiecewiseLinear {
 private:
  static const int kSegmentRangePower = kRangePower - kSegmentsPower;
  static const int kCoeffsCount = 1 << kSegmentsPower;

 public:
  PiecewiseLinear() {
    for (auto i = 0; i < kCoeffsCount; ++i) {
      coeffs_[i] = 0;
    }
    offset = 0;
  }
  int16_t Value(uint16_t x) {
    uint8_t index = x >> kSegmentRangePower;
    uint16_t x1 = index << kSegmentRangePower;
    uint8_t y1 = coeffs_[index];
    uint8_t index_1 = index + 1;

    if (index_1 == kCoeffsCount) {
      index_1 = 0;
    }
    uint8_t y2 = coeffs_[index_1];
    return y1 +
           (((static_cast<int16_t>(x) - static_cast<int16_t>(x1)) *
             (static_cast<int16_t>(y2) - static_cast<int16_t>(y1))) >>
            kSegmentRangePower) -
           offset;
  }

  uint8_t offset;
  uint8_t coeffs_[kCoeffsCount];
};