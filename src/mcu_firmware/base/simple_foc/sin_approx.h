#pragma once

#include "stdint.h"

// https://www.nullhardware.com/blog/fixed-point-sine-and-cosine-for-embedded-systems/

/*
Implements the 5-order polynomial approximation to sin(x).
@param i   angle (with 2^15 units/circle)
@return    16 bit fixed point Sine value (4.12) (ie: +4096 = +1 & -4096 = -1)

The result is accurate to within +- 1 count. ie: +/-2.44e-4.
*/

inline int16_t sin15(int16_t i) {
  /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which
   * is the region of the curve fit). */
  /* ------------------------------------------------------------------- */
  i <<= 1;
  uint8_t c = i < 0;  // set carry for output pos/neg

  // flip input value to corresponding value in range [0..8192)
  if (i == (i | 0x4000)) i = (1 << 15) - i;
  i = (i & 0x7FFF) >> 1;
  /* ------------------------------------------------------------------- */

  /* The following section implements the formula:
   = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1
  * y]) * 2^(a-q) Where the constants are defined as follows: */
  const unsigned long AA1 = 3370945099UL;
  const unsigned long BB1 = 2746362156UL;
  const unsigned long CC1 = 292421UL;
  enum { n = 13, p = 32, q = 31, r = 3, a = 12 };

  uint32_t y = (CC1 * ((uint32_t)i)) >> n;
  y = BB1 - (((uint32_t)i * y) >> r);
  y = (uint32_t)i * (y >> n);
  y = (uint32_t)i * (y >> n);
  y = AA1 - (y >> (p - q));
  y = (uint32_t)i * (y >> n);
  y = (y + (1UL << (q - a - 1))) >> (q - a);  // Rounding

  return c ? -y : y;
}

//Cos(x) = sin(x + pi/2)
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))