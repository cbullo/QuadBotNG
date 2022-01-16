#pragma once

#include "Arduino.h"
#include "FOCMotor.h"
#include "binary_commands.h"
#include "common/lowpass_filter.h"
#include "pid.h"

// callback function pointer definiton
typedef void (*BinaryCommandCallback)(
    uint8_t*);  //!< command callback function pointer

/**
 * Commander class implementing string communication protocol based on IDvalue
 * (ex AB5.321 - command id `A`, sub-command id `B`,value `5.321`)
 *
 *  - This class can be used in combination with HardwareSerial instance which
 * it would read and write or it can be used to parse strings that have been
 * received from the user outside this library
 *  - Commander class implements command protocol for few standard components of
 * the SimpleFOC library
 *     - FOCMotor
 *     - PIDController
 *     - LowPassFilter
 *  - Commander also provides a very simple command > callback interface that
 * enables user to attach a callback function to certain command id - see
 * function add()
 */
class BinaryCommander {
 public:
  /**
   * Default constructor receiving a serial interface that it uses to output the
   * values to Also if the function run() is used it uses this serial instance
   * to read the serial for user commands
   *
   * @param serial - Serial com port instance
   * @param eol - the end of line sentinel character
   * @param echo - echo last typed character (for command line feedback)
   */
  BinaryCommander(Stream& serial);
  BinaryCommander();

  /**
   * Function reading the serial port and firing callbacks that have been added
   * to the commander once the user has requested them - when he sends the
   * command
   *
   *  - It has default commands (the letters can be changed in the commands.h
   * file)
   *    '@' - Verbose mode
   *    '#' - Number of decimal places
   *    '?' - Scan command - displays all the labels of attached nodes
   */
  void run();
  /**
   * Function reading the string of user input and firing callbacks that have
   * been added to the commander once the user has requested them - when he
   * sends the command
   *
   *  - It has default commands (the letters can be changed in the commands.h
   * file)
   *    '@' - Verbose mode
   *    '#' - Number of decimal places
   *    '?' - Scan command - displays all the labels of attached nodes
   *
   * @param reader - temporary stream to read user input
   * @param eol - temporary end of line sentinel
   */
  void run(Stream& reader);
  /**
   * Function reading the string of user input and firing callbacks that have
   * been added to the commander once the user has requested them - when he
   * sends the command
   *
   *  - It has default commands (the letters can be changed in the commands.h
   * file)
   *    '@' - Verbose mode
   *    '#' - Number of decimal places
   *    '?' - Scan command - displays all the labels of attached nodes
   *
   * @param user_input - string of user inputs
   */
  // void run(std::array<uint8_t>& input);

  /**
   *  Function adding a callback to the commander with the command id
   * @param id         - char command letter
   * @param onCommand  - function pointer void function(char*)
   * @param label      - string label to be displayed when scan command sent
   */
  void add(uint8_t id, CommandCallback onCommand);

  // monitoring functions
  Stream* com_port = nullptr;  //!< Serial terminal variable if provided

  void motor(FOCMotor* motor, uint8_t* user_cmd);

  /**
   * Low pass fileter command interface
   *  - It only has one property - filtering time constant Tf
   *  - It can be get by sending 'F'
   *  - It can be set by sending 'Fvalue' - (ex. F0.01 for settin Tf=0.01)
   */
  void lpf(LowPassFilter* lpf, uint8_t* user_cmd);
  /**
   * PID controller command interface
   *  - It has several paramters (the letters can be changed in the commands.h
   * file)
   *     - P gain       - 'P'
   *     - I gain       - 'I'
   *     - D gain       - 'D'
   *     - output ramp  - 'R'
   *     - output limit - 'L'
   *  - Each of them can be get by sening the command letter -(ex. 'D' - to get
   * the D gain)
   *  - Each of them can be set by sending 'IDvalue' - (ex. I1.5 for setting
   * I=1.5)
   */
  void pid(PIDController* pid, uint8_t* user_cmd);

 private:
  // Subscribed command callback variables
  BinaryCommandCallback call_list[12];  //!< array of command callback pointers
  char call_ids[12];                    //!< added callback commands

  // helping variable for serial communication reading
  uint8_t received_cmd[PRIMARY_COMMANDS_COUNT] = {
      0};           //!< so far received user message - waiting for newline
  int rec_cnt = 0;  //!< number of characters received

  // serial printing functions
  /**
   *  print the string message only if verbose mode on
   *  @param message - message to be printed
   */
  void sendString(StringType level, const char* message);
  void sendString(StringType level, const __FlashStringHelper* message);
  
  void send(const float number);
  void send(const uint16_t number);
  void send(const uint8_t number);
};
