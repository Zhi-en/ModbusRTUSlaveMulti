/*
  ModbusRTUSlave64Example

  This library is a modification of ModbusRTUSlave library (https://github.com/CMB27/ModbusRTUSlave).
  The main functions and functions involving coils and discretes are unchanged.
  The functions involving holdings and inputs are modified to take in custom Modbus16bit datatype which is a combination of 4 16bit registers instead of 16bit.

  This program has been succsessfully tested with the following boards:
  - Arduino Nano ESP32

  Other components required:
  - MAX485 RS485 transeiver module (or similar)

  Problems might be encountered when using boards with 32-bit doubles instead of 64-bit doubles. To check this, use sizeof(double).

  !WARNING
  When connecting boards using UART, as described in the circuit below, the logic level voltages must match (5V or 3.3V).
  If they do not, use a logic level converter, otherwise your 3.3V board could be damaged.
  
  Circuit:
  - RX pin (defined as 3 using Serial1 in this example) to RO pin of the MAX485 board
  - TX pin (defined as 2 using Serial1 in this example) to DI pin of the MAX485 board
  - 3.3V pin to VCC pin of MAX485 board (necessary to power the MAX485 board if not powered by master)
  - GND pin to GND pin of MAX485 board (necessary to power the MAX485 board if not powered by master)
  - Pin 4 (setup as the driver enable pin) to DE and RE pin of MAX485 board (DE and RE pins to be shorted externally, might not be necessary MAX485 alternatives)
  - GND to GND of the master board (not necessary for MAX485 board, might be necessary MAX485 alternatives)
  
  Created: 2024-10-17
  By: Tan Zhi-en
  
*/

#include <ModbusRTUSlaveMulti.h>
#include "Arduino.h"

// define pins
const uint8_t txPin = 5;
const uint8_t rxPin = 4;
const uint8_t dePin = 3;

// setup modbus variables
bool coils[1];
bool discretes[1];
modbus64 holdings[2];
modbus64 inputs[2];

// define input output variables
bool& inp_bool = coils[0];
bool& out_bool = discretes[0];
int64_t& inp_long = holdings[0].INT64;
double& inp_double = holdings[1].FLOAT64;
int64_t& out_long = inputs[0].INT64;
double& out_double = inputs[1].FLOAT64;

// for Arduino Nano ESP32, modify this line according to ModbusRTUSlaveExample for other boards
ModbusRTUSlave64 modbus(Serial1, dePin);

void setup() {
  Serial.begin(115200); // open serial communications with PC

  Serial1.begin(115200, SERIAL_8N1, rxPin, txPin);  // begin modbus serial communication

  modbus.configureCoils(coils, 1);                // bool array of coil values, number of coils
  modbus.configureDiscreteInputs(discretes, 1);   // bool array of discrete input values, number of discrete inputs
  modbus.configureHoldingRegisters(holdings, 2);  // custom 64 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputs, 2);      // custom 64 bit integer array of holding register values, number of holding registers

  modbus.begin(1, 115200);  // slave ID and baudrate

  out_bool = true;
  out_long = 0;
  out_double = 1.1;
}

void loop() {
  out_bool = not out_bool;
  out_long -= 1;
  out_double += 1.1;
  
  modbus.poll();

  Serial.println(inp_bool);
  Serial.println(inp_long);
  Serial.println(inp_double);

  delay(100);
}
