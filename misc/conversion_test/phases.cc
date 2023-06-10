#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>

inline uint16_t normalizeAngle(int32_t in, uint16_t period) {
  while (in < 0) {
    in += period;
  }

  while (in >= period) {
    in -= period;
  }

  return in;
}

int16_t cos9(uint16_t angle) {
  return static_cast<int16_t>(cos((angle / 4096.0) * 2.0 * M_PI) * (1 << 9));
}

// int16_t sin9(uint16_t angle) {
//   return static_cast<int16_t>(sin((angle / 4096.0) * 2.0 * M_PI) * (1 << 9));
// }

int16_t sin15(int16_t i) {
  /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which
   * is the region of the curve fit). */
  /* ------------------------------------------------------------------- */
  i <<= 1;
  uint8_t c = i < 0;  // set carry for output pos/neg

  if (i ==
      (i |
       0x4000))  // flip input value to corresponding value in range [0..8192)
    i = (1 << 15) - i;
  i = (i & 0x7FFF) >> 1;
  /* ------------------------------------------------------------------- */

  /* The following section implements the formula:
   = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1
  * y]) * 2^(a-q) Where the constants are defined as follows:
  */
  enum { A1 = 3370945099UL, B1 = 2746362156UL, C1 = 292421UL };
  enum { n = 13, p = 32, q = 31, r = 3, a = 12 };

  uint32_t y = (C1 * ((uint32_t)i)) >> n;
  y = B1 - (((uint32_t)i * y) >> r);
  y = (uint32_t)i * (y >> n);
  y = (uint32_t)i * (y >> n);
  y = A1 - (y >> (p - q));
  y = (uint32_t)i * (y >> n);
  y = (y + (1UL << (q - a - 1))) >> (q - a);  // Rounding

  y >>= 3;

  return c ? -y : y;
}
void floatFOC(float U, float angle_el, float &Ua, float &Ub, float &Uc) {
  const float voltage_limit = 12;
  const float _PI_3 = M_PI / 3.0;
  const float _SQRT3 = sqrtf(3.0);

  float center;
  int sector;
  float _ca, _sa;

  // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
  // https://www.youtube.com/watch?v=QMSWUMEAejg

  // the algorithm goes
  // 1) Ualpha, Ubeta
  // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
  // 3) angle_el = atan2(Ubeta, Ualpha)
  //
  // equivalent to 2)  because the magnitude does not change is:
  // Uout = sqrt(Ud^2 + Uq^2)
  // equivalent to 3) is
  // angle_el = angle_el + atan2(Uq,Ud)

  float Uout;
  // a bit of optitmisation
  {  // only Uq available - no need for atan2 and sqrt
    Uout = U / voltage_limit;
    // angle normalisation in between 0 and 2pi
    // only necessary if using _sin and _cos - approximation functions
    angle_el = fmodf(angle_el + 0.5 * M_PI, 2.0 * M_PI);
  }
  // find the sector we are in currently
  sector = floor(angle_el / _PI_3) + 1;
  // calculate the duty cycles
  float s1 = sin(sector * _PI_3 - angle_el);
  float s2 = sin(angle_el - (sector - 1.0f) * _PI_3);

  float T1 = _SQRT3 * s1 * Uout;
  float T2 = _SQRT3 * s2 * Uout;

  // two versions possible
  float T0 = 0;  // pulled to 0 - better for low power supply voltage
  if (true) {
    T0 = 1 - T1 - T2;  // modulation_centered around driver->voltage_limit/2
  }

  // calculate the duty cycles(times)
  float Ta, Tb, Tc;
  switch (sector) {
    case 1:
      Ta = T1 + T2 + T0 / 2;
      Tb = T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 2:
      Ta = T1 + T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 3:
      Ta = T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T2 + T0 / 2;
      break;
    case 4:
      Ta = T0 / 2;
      Tb = T1 + T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 5:
      Ta = T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 6:
      Ta = T1 + T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T0 / 2;
      break;
    default:
      // possible error state
      Ta = 0;
      Tb = 0;
      Tc = 0;
  }

  // calculate the phase voltages and center
  Ua = Ta * voltage_limit;
  Ub = Tb * voltage_limit;
  Uc = Tc * voltage_limit;
}

void integerFOC(int16_t U, uint16_t angle_el, int16_t &Ua, int16_t &Ub,
                int16_t &Uc) {
  int16_t sectors[6] = {683, 1365, 2048, 2731, 3413, 4095};

  uint16_t voltage_limit = 12 * (1 << 9);

  int32_t center;
  int16_t _ca, _sa;

  uint16_t sqrt3 = 887;  // 2^9

  // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
  // https://www.youtube.com/watch?v=QMSWUMEAejg

  // the algorithm goes
  // 1) Ualpha, Ubeta
  // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
  // 3) angle_el = atan2(Ubeta, Ualpha)
  //
  // equivalent to 2)  because the magnitude does not change is:
  // Uout = sqrt(Ud^2 + Uq^2)
  // equivalent to 3) is
  // angle_el = angle_el + atan2(Uq,Ud)

  // U /= 256;

  int16_t Uout = U;  // 2^9
  // a bit of optitmisation
  {  // only Uq available - no need for atan2 and sqrt
    // Uout = U / driver->voltage_limit;
    // angle normalisation in between 0 and 2pi
    // only necessary if using _sin and _cos - approximation functions
    angle_el = normalizeAngle(angle_el + 1024, 4096);
  }

  // calculate the duty cycles(times)
  int32_t Ta, Tb, Tc;
  if (angle_el <= sectors[0]) {
    int16_t s1 = sin15((sectors[0] - angle_el) << 3);
    int16_t s2 = sin15(angle_el << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
  } else if (angle_el <= sectors[1]) {
    int16_t s1 = sin15((sectors[1] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[0]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;

  } else if (angle_el <= sectors[2]) {
    int16_t s1 = sin15((sectors[2] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[1]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
  } else if (angle_el <= sectors[3]) {
    int16_t s1 = sin15((sectors[3] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[2]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
  } else if (angle_el <= sectors[4]) {
    int16_t s1 = sin15((sectors[4] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[3]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }

    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
  } else if (angle_el <= sectors[5]) {
    int16_t s1 = sin15((sectors[5] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[4]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }

    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
  } else {
    // possible error state
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  // calculate the phase voltages and center
  Ua = Ta * voltage_limit / (1 << 9);
  Ub = Tb * voltage_limit / (1 << 9);
  Uc = Tc * voltage_limit / (1 << 9);
}

void floatSinFOC(float U, float angle_el, float &Ua, float &Ub, float &Uc) {
  float _ca = cosf(angle_el);
  float _sa = sinf(angle_el);
  // Inverse park transform
  float Ualpha = -_sa * U;  // -sin(angle) * Uq;
  float Ubeta = +_ca * U;   //  cos(angle) * Uq;

  // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
  float center = 12.0 / 2;
  // Clarke transform
  Ua = Ualpha + center;
  Ub = -0.5f * Ualpha + sqrtf(3.0) / 2.0 * Ubeta + center;
  Uc = -0.5f * Ualpha - sqrtf(3.0) / 2.0 * Ubeta + center;
}

void integerSinFOC(int16_t U, int16_t angle_el, int32_t &Ua, int32_t &Ub,
                   int32_t &Uc) {
  // Sinusoidal PWM modulation
  // Inverse Park + Clarke transformation

  // angle normalization in between 0 and 2pi
  // only necessary if using _sin and _cos - approximation functions
  angle_el = normalizeAngle(angle_el, 4096);
  int32_t _ca = sin15(normalizeAngle(1024 - angle_el, 4096) << 3);
  int32_t _sa = sin15(angle_el << 3);
  // Inverse park transform
  int32_t Ualpha = -_sa * U / (1 << 9);
  int32_t Ubeta = _ca * U / (1 << 9);

  // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
  int32_t center = (12 * 512) / 2;
  // Clarke transform
  Ua = Ualpha + center;
  Ub = -Ualpha / 2 + (443 * Ubeta) / 512 + center;
  Uc = -Ualpha / 2 - (443 * Ubeta) / 512 + center;
}

int16_t electricalAngle(uint16_t a) {
  // if no sensor linked return previous value ( for open loop )

  int32_t pos = a;

  return normalizeAngle((7) * pos - 0, 4096);

  // int32_t pos = a;

  // pos = normalizeAngle(pos, 4096);
  // pos *= 7;
  // uint16_t normalized_pos = normalizeAngle(pos, 4096);
  // return normalized_pos;

  // return normalized_pos;
}

#define _constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define N_SIN 4096
#define N_SIN_4 1024
#define N_SIN_3 1365
#define N_SIN_2_3 2731

void integerSinFOC2(int16_t U, uint16_t angle_el, int32_t &Ua, int32_t &Ub,
                    int32_t &Uc) {
  int32_t pwm_a;
  int32_t pwm_b;
  int32_t pwm_c;

  pwm_a = sin15(angle_el << 3);
  pwm_b = sin15(normalizeAngle(angle_el + N_SIN_3, N_SIN) << 3);
  pwm_c = sin15(normalizeAngle(angle_el + N_SIN_2_3, N_SIN) << 3);

  uint16_t power = abs(U);
  int32_t max_voltage = (12 * (1 << 9)); 
  int32_t center = max_voltage / 2 - 1;

  // apply power factor
  pwm_a = power * pwm_a;
  pwm_a = pwm_a >> 10;
  pwm_a += center;

  pwm_b = power * pwm_b;
  pwm_b = pwm_b >> 10;
  pwm_b += center;

  pwm_c = power * pwm_c;
  pwm_c = pwm_c >> 10;
  pwm_c += center;

  // pwm_a = std::max(0, std::min(pwm_a, 12 * 512));
  // pwm_b = std::max(0, std::min(pwm_b, 12 * 512));
  // pwm_c = std::max(0, std::min(pwm_c, 12 * 512));

  // uint16_t vps = 8 * 512;
  // vps >>= 9;

  //uint16_t uua = pwm_a;
  //uint16_t uub = pwm_b;
  //uint16_t uuc = pwm_c;

  Ua = pwm_a / (2 * (max_voltage >> 9));
  Ub = pwm_b / (2 * (max_voltage >> 9));
  Uc = pwm_c / (2 * (max_voltage >> 9));
}

int main() {
  std::ofstream fout("output.txt");

  for (int i = 0; i < 4096; ++i) {
    //   float angle_f = 2.0 * M_PI * (i / 4096.0);
    // int16_t elec_angle = electricalAngle(i);
    int16_t elec_angle = i;
    //   float elec_angle_f = 2.0 * M_PI * (elec_angle / 4096.0);
    //   // std::cout << "Angle: " << i << " / " << angle_f << " / " <<
    //   elec_angle
    //   //           << " / " << elec_angle_f << std::endl;
    //   // std::cout << "Elec. angle: " << elec_angle << std::endl;

    //   float Ua_f;
    //   float Ub_f;
    //   float Uc_f;

    //   floatSinFOC(6.0, elec_angle_f, Ua_f, Ub_f, Uc_f);
    //   // std::cout << "Us float: " << Ua_f << " " << Ub_f << " " << Uc_f
    //   //           << std::endl;

    //   Ua_f = _constrain(Ua_f, 0.0f, 12.f);
    //   Ub_f = _constrain(Ub_f, 0.0f, 12.f);
    //   Uc_f = _constrain(Uc_f, 0.0f, 12.f);
    //   // calculate duty cycle
    //   // limited in [0,1]
    //   float dc_a_f = _constrain(Ua_f / 12.f, 0.0f, 1.0f);
    //   float dc_b_f = _constrain(Ub_f / 12.f, 0.0f, 1.0f);
    //   float dc_c_f = _constrain(Uc_f / 12.f, 0.0f, 1.0f);

    //   // std::cout << "Dc float: " << dc_a_f * 255.0<< " " << dc_b_f *
    //   255.0<< " "
    //   // << dc_c_f * 255.0
    //   //           << std::endl;

    int32_t Ua;
    int32_t Ub;
    int32_t Uc;

    //   int32_t vl = 12 * 512 - 1;

    //   //integerSinFOC(6 * (1 << 9), elec_angle, Ua, Ub, Uc);

    // integerSinFOC2(5 * (1 << 9), elec_angle, Ua, Ub, Uc);

    //   Ua = std::max((int32_t)0, std::min(Ua, vl));
    //   Ub = std::max((int32_t)0, std::min(Ub, vl));
    //   Uc = std::max((int32_t)0, std::min(Uc, vl));

    //   uint16_t vps = 12 * 512;
    //   vps >>= 8;

    //   uint16_t uua = Ua;
    //   uint16_t uub = Ub;
    //   uint16_t uuc = Uc;

    //   uint8_t dc_a = uua / vps;
    //   uint8_t dc_b = uub / vps;
    //   uint8_t dc_c = uuc / vps;

    //   // std::cout << "Us integer: " << (float)Ua / (float)(1 << 9) << " "
    //   //           << (float)Ub / (float)(1 << 9) << " "
    //   //           << (float)Uc / (float)(1 << 9) << std::endl;
    //   fout << (int)dc_a << " " << (int)dc_b << " " << (int)dc_c << std::endl;

    //   //   std::cout << std::endl;
    // }

    int16_t offset = N_SIN_4;
    uint16_t angle_to_set = normalizeAngle(i + offset, N_SIN);
    integerSinFOC2(8 * (1 << 9), angle_to_set, Ua, Ub, Uc);

    uint16_t angle_corrected =
      static_cast<uint16_t>(4095 + 2) & (0xFFF);

    std::cout << angle_corrected << std::endl;

    fout << i << " " << Ua << " " << Ub << " " << Uc << std ::endl;
  }
  return 0;
}