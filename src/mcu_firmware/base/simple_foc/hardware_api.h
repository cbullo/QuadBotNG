#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

// #include "../common/foc_utils.h"
// #include "../common/time_utils.h"

/**
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pinA pinA bldc driver
 * @param pinB pinB bldc driver
 * @param pinC pinC bldc driver
 */
void _configure3PWM(long pwm_frequency, const int pinA, const int pinB,
                    const int pinC);

/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * @param dc_a  duty cycle phase A [0, 255]
 * @param dc_b  duty cycle phase B [0, 255]
 * @param dc_c  duty cycle phase C [0, 255]
 * @param pinA  phase A hardware pin number
 * @param pinB  phase B hardware pin number
 * @param pinC  phase C hardware pin number
 */
void _writeDutyCycle3PWM(uint8_t dc_a, uint8_t dc_b, uint8_t dc_c, int pinA,
                         int pinB, int pinC);

#endif