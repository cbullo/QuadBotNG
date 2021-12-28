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

#define POLE_PAIR_NUMBER 7
#define PHASE_RESISTANCE 5  // ohm

CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

Commander commander = Commander(Serial, '\n', false);

void on_calib_data1(char *cmd) {
  switch (cmd[0]) {
    case 'C': {
      uint8_t index = 0;
      if (cmd[1] == '1') {
        index += 10;
      }
      if ('0' <= cmd[2] && cmd[2] <= '9') {
        index += cmd[2] - '0';
      }
      if (index <= 15) {
        float value = 0;
        commander.scalar(&value, &cmd[3]);
        sensor.linearization_.coeffs_[index] = value;
      }
      break;
    }
    case 'O': {
      float value = 0;
      commander.scalar(&value, &cmd[1]);
      sensor.linearization_.offset = value;
      break;
    }
  }
}



// BLDC motor & driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 3, 6);
BLDCMotor motor = BLDCMotor(POLE_PAIR_NUMBER);

void Critical(){};

float mean = 0.0;
float prev_mean = 0.0;
float stDevSum = 0;

const int sample_points = 256;
const int samples_margin = 0;
int16_t current_sample = 0;
int test_direction = 1;

extern float temperature[2];

int phase = 0;

void on_calib_plus(char *cmd);
void on_calib_minus(char *cmd);

void Initialize() {
  // driver config
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 12;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = motor.voltage_sensor_align;
  motor.velocity_limit = 100;  // pole_pairs_number * 4

  sensor.Activate();
  sensor.init();
  // motor.linkSensor(&sensor);

  motor.useMonitoring(Serial);
  motor.init();

  motor.disable();

  commander.add('E', on_calib_data1, "encoder 1");
  commander.add('P', on_calib_plus, "plus");
  commander.add('M', on_calib_minus, "minus");

  Serial.println(F("INIT"));
}

const uint8_t N = 4;
uint16_t readings[N];

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

#define MOTOR_ON_PER_SAMPLE_TIME 500  // us
#define MOTOR_OFF 10000              // us
int current_reading = 0;

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

void on_calib_plus(char *cmd) {
  phase = 1;
  test_direction = 1;
  current_sample = -samples_margin;
  Serial.println(F("STARTING CALIB IN PLUS"));
  motor.enable();
}

void on_calib_minus(char *cmd) {
  phase = 1;
  test_direction = -1;
  current_sample = sample_points + samples_margin;
  Serial.println(F("STARTING CALIB IN MINUS"));
  motor.enable();
}

void Tick() {
  commander.run();

  bool can_run = true;
  if (wait_until != 0) {
    if (micros() < wait_until) {
      can_run = false;
    }
  }

  if (!can_run) {
  } else if (phase == 0) {
  } else if (phase == 1) {
    // Read N samples
    if (-samples_margin <= current_sample &&
        current_sample <= sample_points + samples_margin) {
      float motor_angle = current_sample * (_2PI / sample_points);
      motor.move(motor_angle);
      if (WaitFor(MOTOR_ON_PER_SAMPLE_TIME)) {
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
    } else {
      phase = 2;
      Serial.println(F("FINISHED"));
    }
  } else if (phase == 12) {
    // Read samples and compare with average. Stop if diff <= 2

    float motor_angle = current_sample * (_2PI / sample_points);
    motor.move(motor_angle);
    if (WaitFor(MOTOR_ON_PER_SAMPLE_TIME)) {
      sensor.update();
      uint16_t current_sensor_angle =
          round((sensor.getSensorAngle() / _2PI) * 4096);
      uint16_t average_sensor_angle = GetAverageReading();

      if (abs(current_sensor_angle - average_sensor_angle) <= 3) {
        // Serial.print(current_sample);
        // Serial.print(",");
        Serial.print(average_sensor_angle);
        Serial.print(",");
        //Serial.println(temperature[1]);
        current_sample += test_direction;
        phase = 13;
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
  } else if (phase == 13) {
    motor.disable();
    if (WaitFor(MOTOR_OFF)) {
      phase = 1;
      ClearWait();
      motor.enable();
    }
  } else {
    motor.voltage_limit = 0;
    motor.current_limit = 0;
    motor.disable();
  }

  sensor.update();
  // motor.move();
}
