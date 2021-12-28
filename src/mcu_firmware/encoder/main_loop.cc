#include "SimpleFOC.h"
#include "base/custom_magnetic_sensor_i2c.h"

// Things that need to be guaranteed to get called. These are guarded with WDT
void Critical() {}

CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

Commander commander = Commander(Serial, '\n', true);

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

unsigned long next_sensor_read;
void Initialize() {
  sensor.Activate();
  sensor.init();
  _delay(1000);
  Serial.println(F("encoder_angle,encoder_velocity"));
  next_sensor_read = millis();

  commander.add('E', on_calib_data1, "encoder 1");
}

#define SENSOR_READ_PERIOD 100;  // ms
void Tick() {
  sensor.update();
  commander.run();
  if (millis() >= next_sensor_read) {
    //Serial.print(round(sensor.getAngle() * kRadToEnc));
    //Serial.print("\t");
    //Serial.println(sensor.getVelocity());
    next_sensor_read += SENSOR_READ_PERIOD;
  }
}
