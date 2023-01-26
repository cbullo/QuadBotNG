/**
 * This measures how closely sensor and electrical angle agree and how much your
 * motor is affected by 'cogging'. It can be used to investigate how much non
 * linearity there is between what we set (electrical angle) and what we read
 * (sensor angle) This non linearity could be down to magnet placement, coil
 * winding differences or simply that the magnetic field when travelling through
 * a pole pair is not linear An alignment error of ~10 degrees and cogging of ~4
 * degrees is normal for small gimbal. The following article is an interesting
 * read
 * https://hackaday.com/2016/02/23/anti-cogging-algorithm-brings-out-the-best-in-your-hobby-brushless-motors/
 */

#include <avr/wdt.h>

#include "Arduino.h"
#include "SimpleFOC.h"
#include "base/custom_magnetic_sensor_i2c.h"

CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A0, A1);

void Critical(){};

#define SENSOR_READ_DELAY 5000  // us
#define MOTOR_OFF 10000         // us
const int sample_points = 64;

int current_reading = 0;
int phase = -1;
int16_t current_sample = 0;

const uint8_t N = 16;
uint16_t readings[N];

void Initialize() {
  sensor.activate();
  sensor.init();

  Serial.println(F("INIT1"));
}

void ShiftReadings() {
  for (int i = 0; i < N - 1; ++i) {
    readings[i] = readings[i + 1];
  }
}

uint16_t GetAverageReading() {
  uint16_t x = 0;
  uint16_t y = 0;

  for (int i = 0; i < N; i++) {
    x += readings[i] / N;
    int b = readings[i] % N;
    if (y >= N - b) {
      x++;
      y -= N - b;
    } else {
      y += b;
    }
  }

  return x + (y >= N / 2 ? 1 : 0);
}

unsigned long wait_until = 0;
bool WaitFor(unsigned long period_us) {
  if (wait_until == 0) {
    wait_until = micros() + period_us;
    return false;
  } else {
    return micros() >= wait_until;
  }
}
void ClearWait() { wait_until = 0; }

void Tick() {
  bool can_run = true;
  if (wait_until != 0) {
    if (micros() < wait_until) {
      can_run = false;
    }
  }

  if (!can_run) {
  } else if (phase == -1) {
    Serial.println();
    Serial.print(current_sample);
    Serial.print(",");
    phase = 0;
  } else if (phase == 0) {
    if (Serial.available()) {
      int val = Serial.read();
      if (val == '+') {
        phase = 1;
      } else {
        return;
      }
    } else {
      return;
    }
  } else if (phase == 1) {
    // Read N samples
    if (WaitFor(SENSOR_READ_DELAY)) {
      // 4096 is encoder cpr
      sensor.update();
      uint16_t sensor_angle = round((sensor.getSensorAngle() / _2PI) * 4096);
      readings[current_reading] = sensor_angle;
      ++current_reading;
      if (current_reading == N) {
        phase = 12;
      }
      ClearWait();
    }

  } else if (phase == 12) {
    // Read samples and compare with average. Stop if diff <= 2
    if (WaitFor(SENSOR_READ_DELAY)) {
      sensor.update();
      uint16_t current_sensor_angle =
          round((sensor.getSensorAngle() / _2PI) * 4096);
      uint16_t average_sensor_angle = GetAverageReading();

      if (abs(current_sensor_angle - average_sensor_angle) <= 3) {
        Serial.print(average_sensor_angle);
        current_sample += 1;

        if (current_sample == sample_points) {
          phase = 2;
          Serial.println(F("FINISHED"));
          return;
        }
        phase = -1;
        current_reading = 0;
      } else {
        ShiftReadings();
        readings[N - 1] = current_sensor_angle;
        // Serial.print("STABILIZING ");
        // Serial.print(average_sensor_angle);
        // Serial.print(",");
        // Serial.println(current_sensor_angle);
      }
      ClearWait();
    }
  } else {
  }

  sensor.update();
  // motor.move();
}
