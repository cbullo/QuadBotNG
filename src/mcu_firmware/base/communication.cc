// #include "BLDCMotor.h"
// #include "communication/Commander.h"
// #include "custom_magnetic_sensor_i2c.h"
// #include "drivers/BLDCDriver3PWM.h"

// static BinaryCommander commander = Commander(Serial);

// extern BLDCMotor motors[2];
// extern BLDCDriver3PWM drivers[2];
// extern CustomMagneticSensorI2C sensors[2];

// void ProcessStateCommand(char *cmd);

// void on_sensor(char *cmd) {
//   uint8_t sensor_index = cmd[0] - '0';
//   switch (cmd[1]) {
//     case 'C': {
//       uint8_t index = 0;
//       if (cmd[2] == '1') {
//         index += 10;
//       }
//       if ('0' <= cmd[3] && cmd[3] <= '9') {
//         index += cmd[3] - '0';
//       }
//       if (index <= 15) {
//         float value = 0;
//         commander.scalar(&value, &cmd[4]);
//         sensors[sensor_index].linearization_.coeffs_[index] = value;
//       }
//       break;
//     }
//     case 'O': {
//       float value = 0;
//       commander.scalar(&value, &cmd[2]);
//       sensors[sensor_index].linearization_.offset = value;
//       break;
//     }
//   }
// }

// void on_state(char *cmd) {
//   if (cmd[0] == 'Q') {
//     Serial.println("RESETTING");
//     // Reset requested - wait for WDT to kick in
//     while (1)
//       ;
//   }
//   ProcessStateCommand(cmd);
// }

// // void InitCommunication() {
// //   commander.add('M', on_motor, "on motor");
// //   commander.add('S', on_sensor, "on sensor");
// //   commander.add('V', on_voltage, "on voltage");
// //   // commander.add('C', on_stop, "on state");
// // }



// SerialCommunication::SerialCommunication()
//     : synced_(SerialCommunication::SyncState::kNotSynced) {

//     }

// SerialCommunication::Tick() {
//   switch (synced) {
//   case SyncState::kNotSynced:
//     if (last_received_time - millis() > 5000);
//     break;
//   case SyncState::kSyncing:
//     break;
//   case SyncState::kSynced:
//   }
// }