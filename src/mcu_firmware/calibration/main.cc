#include "Arduino.h"
#include "SimpleFOC.h"
#include "../../custom_magnetic_sensor_i2c.h"

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t
// _angle_register_msb)
//  chip_address  I2C chip address
//  bit_resolution  resolution of the sensor
//  angle_register_msb  angle read register msb
//  bits_used_msb  number of used bits in msb register
//
// make sure to read the chip address and the chip angle register msb value from
// the datasheet also in most cases you will need external pull-ups on SDA and
// SCL lines!!!!!
//
// For AS5058B
// MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// Example of AS5600 configuration
CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 3, 6);
BLDCMotor motor = BLDCMotor(7);

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
void testAlignmentAndCogging(int direction) {
  motor.move(0 /*_3PI_2 / motor.pole_pairs*/);
  _delay(200);

  float initialAngle = sensor.getAngle();

  const int shaft_rotation =
      720;  // 720 deg test - useful to see repeating cog pattern
  int sample_count =
      int(shaft_rotation * motor.pole_pairs);  // test every electrical degree

  float stDevSum = 0;

  float mean = 0.0;
  float prev_mean = 0.0;

  for (int i = 0; i < sample_count; i++) {
    float electricAngle = (float)direction * (float)i / motor.pole_pairs;
    // move and wait
    motor.move(/*_3PI_2 / motor.pole_pairs +*/ electricAngle * PI / 180);
    _delay(5);

    // electricAngle *= motor.pole_pairs;

    // measure
    float sensorAngle = motor.sensor_direction *
                        (sensor.getAngle() - initialAngle) * 180.f / PI;
    // float sensorElectricAngle = sensorAngle;
    float electricAngleError = electricAngle - sensorAngle;

    // plot this - especially electricAngleError
    Serial.print(electricAngle);
    Serial.print("\t");
    Serial.print(sensorAngle);
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

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // driver config
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 7;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = motor.voltage_sensor_align;

  sensor.activate();
  sensor.init();
  motor.linkSensor(&sensor);

  motor.useMonitoring(Serial);
  motor.init();

  // motor.initFOC(4.56f + 0.29f);

  double mean_zero_offset = 0.f;
  for (int i = 0; i < 10; ++i) {
    motor.zero_electric_angle = NOT_SET;
    motor.initFOC();
    mean_zero_offset += 0.1 * motor.zero_electric_angle;
  }
  Serial.print(F("Average zero offset: "));
  Serial.println(mean_zero_offset);

  //testAlignmentAndCogging(-1);

  motor.move(0);
  Serial.println(F("Press any key to test in CCW direction"));
  while (!Serial.available()) {
  }

  //testAlignmentAndCogging(-1);

  Serial.println(F("Complete"));

  motor.voltage_limit = 0;
  motor.move(0);
  while (true)
    ;  // do nothing;
}

void loop() {}

int main() {
  setup();
  while (true) {
    loop();
  }
  return 0;
}
