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

// BLDC motor & driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 3, 6);
BLDCMotor motor = BLDCMotor(POLE_PAIR_NUMBER);

void Critical(){};

void testAlignmentAndCogging(int direction) {
  motor.move(0);
  _delay(200);

  sensor.update();
  float initialAngle = sensor.getAngle();

  const int shaft_rotation =
      720;  // 720 deg test - useful to see repeating cog pattern
  int sample_count =
      int(shaft_rotation * motor.pole_pairs);  // test every electrical degree

  float stDevSum = 0;

  float mean = 0.0;
  float prev_mean = 0.0;

  for (int i = 0; i < sample_count; i++) {
    float electricAngle = (float)direction * (float)i;
    //(float)direction * i * motor.pole_pairs * shaft_rotation / sample_count;
    // move and wait
    motor.move((electricAngle * PI / 180) / motor.pole_pairs);
    _delay(5);

    // measure
    sensor.update();
    float sensorAngle = ((sensor.getAngle() - initialAngle) * 180.f / PI);
    float sensorElectricAngle = sensorAngle * motor.pole_pairs;
    float electricAngleError = electricAngle - sensorElectricAngle;

    // plot this - especially electricAngleError
    Serial.print(electricAngle);
    Serial.print("\t");
    Serial.print(sensorElectricAngle);
    Serial.print("\t");
    Serial.println(electricAngleError);

    // use knuth standard deviation algorithm so that we don't need an array too
    // big for an Uno
    prev_mean = mean;
    mean = mean + (electricAngleError - mean) / (i + 1);
    stDevSum = stDevSum +
               (electricAngleError - mean) * (electricAngleError - prev_mean);
  }

  Serial.println();
  Serial.println(F("ALIGNMENT AND COGGING REPORT"));
  Serial.println();
  Serial.print(F("Direction: "));
  Serial.println(direction);
  Serial.print(F("Mean error (alignment): "));
  Serial.print(mean);
  Serial.println(" deg (electrical)");
  Serial.print(F("Standard Deviation (cogging): "));
  Serial.print(sqrt(stDevSum / sample_count));
  Serial.println(F(" deg (electrical)"));
  Serial.println();
  Serial.println(
      F("Plotting 3rd column of data (electricAngleError) will likely show "
        "sinusoidal cogging pattern with a frequency of 4xpole_pairs per "
        "rotation"));
  Serial.println();
}

void Initialize() {
  // driver config
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 4;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = motor.voltage_sensor_align;

  sensor.activate();
  sensor.init();
  motor.linkSensor(&sensor);

  motor.useMonitoring(Serial);
  motor.init();

  wdt_disable();
}

bool tested = false;
void Tick() {
  if (!tested) {
    testAlignmentAndCogging(1);

    Serial.println(F("Complete"));

    motor.voltage_limit = 0;
    motor.current_limit = 0;
    motor.disable();
    wdt_enable(WDTO_1S);
    tested = true;
  }
}
