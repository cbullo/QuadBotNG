#include "Arduino.h"

template <uint8_t kRangePower, uint8_t kSegmentsPower>
class PiecewiseLinear {
 private:
  static const uint8_t kSegmentRangePower = kRangePower - kSegmentsPower;
  static const uint8_t kCoeffsCount = 1 << kSegmentsPower;

 public:
  PiecewiseLinear() {
    for (auto i = 0; i < kCoeffsCount; ++i) {
      coeffs_[i] = 0;
    }
    offset_ = 0;
  }
  int16_t Value(uint16_t x) {
    uint8_t index = x >> kSegmentRangePower;
    uint16_t x1 = index << kSegmentRangePower;
    uint8_t y1 = coeffs_[index];
    uint8_t index_1 = (index == kCoeffsCount - 1) ? 0 : index + 1;
    uint8_t y2 = coeffs_[index_1];
    return y1 +
           (((static_cast<int16_t>(x) - static_cast<int16_t>(x1)) *
             (static_cast<int16_t>(y2) - static_cast<int16_t>(y1))) >>
            kSegmentRangePower) -
           offset_;
  }

  uint8_t offset_;
  uint8_t coeffs_[kCoeffsCount];
};