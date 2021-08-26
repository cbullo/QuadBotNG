#include "Arduino.h"
#include "SimpleFOC.h"
#include "custom_magnetic_sensor_i2c.h"
#include "drivers/BLDCDriver3PWM.h"

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
CustomMagneticSensorI2C sensor1 = CustomMagneticSensorI2C(AS5600_I2C, A0, A1);
CustomMagneticSensorI2C sensor2 = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(5, 3, 6);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(9, 11, 10);

BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);

volatile bool stopped = true;

void do_enable() {
  stopped = false;
}

void do_disable() {
  stopped = true;
}

void setup() {


  // monitoring port
  Serial.begin(57600);

  // configure i2C
  Wire.setClock(400000);
  
  // initialise magnetic sensor hardware
  sensor1.activate();
  sensor1.init();

  // Serial.println("Sensor 1 ready");
  // _delay(1000);

  sensor2.activate();
  sensor2.init();

  // Serial.println("Sensor 2 ready");
  // _delay(1000);

  driver1.voltage_power_supply = 28;
  driver2.voltage_power_supply = 28;

  driver1.voltage_limit = 4;
  driver2.voltage_limit = 4;

  driver1.init();
  driver2.init();

  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.controller = MotionControlType::velocity_openloop;
  motor2.controller = MotionControlType::velocity_openloop;

  motor1.target = 6.28;
  motor2.target = 6.28;

  motor1.velocity_limit = 10;
  motor2.velocity_limit = 10;

  motor1.init();
  motor2.init();

  driver1.enable();
  driver2.enable();
   _delay(1000);
}

void loop() {
  if (stopped) {
    motor1.disable();
    motor2.disable();
  }

  // display the angle and the angular velocity to the terminal
  // Serial.print(sensor1.getAngle());
  // Serial.print("\t");
  // Serial.print(sensor1.getVelocity());
  // Serial.print("\t");
  // Serial.print(sensor2.getAngle());
  // Serial.print("\t");
  // Serial.println(sensor2.getVelocity());

  motor1.move();
  motor2.move();

  //motor1.monitor();
  //motor2.monitor();


}