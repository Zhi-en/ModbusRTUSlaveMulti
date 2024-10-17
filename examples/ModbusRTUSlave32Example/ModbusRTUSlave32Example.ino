/*
  ModbusRTUSlave32Example

  This library is a modification of ModbusRTUSlave library (https://github.com/CMB27/ModbusRTUSlave).
  The main functions and functions involving coils and discretes are unchanged.
  The functions involving holdings and inputs are modified to take in custom Modbus16bit datatype which is a combination of 4 16bit registers instead of 16bit.

  This program has been succsessfully tested with the following boards:
  - Arduino Nano ESP32

  Other components required:
  - MAX485 RS485 transeiver module (or similar)

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

// define pins
const uint8_t txPin = 2;
const uint8_t rxPin = 3;
const uint8_t dePin = 4;

// setup modbus variables
bool coils[2];
bool discretes[2];
modbus32 holdings[2];
modbus32 inputs[2];

// define input output variables
bool& inp_bool_1 = coils[0];
bool& inp_bool_2 = coils[1];
bool& out_bool_1 = discretes[0];
bool& out_bool_2 = discretes[0];
int32_t& inp_int = holdings[0]._int;
float& inp_float = holdings[1]._float;
int32_t& out_int = inputs[0]._int;
float& out_float = inputs[1]._float;

// for Arduino Nano ESP32, modify this line according to ModbusRTUSlaveExample for other boards
ModbusRTUSlave32 modbus(Serial1, dePin);

void setup() {
  // Open serial communications with PC
  Serial.begin(115200);

  modbus.configureCoils(coils, 2);                // bool array of coil values, number of coils
  modbus.configureDiscreteInputs(discretes, 2);   // bool array of discrete input values, number of discrete inputs
  modbus.configureHoldingRegisters(holdings, 2);  // custom 64 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputs, 2);      // custom 64 bit integer array of holding register values, number of holding registers

  modbus.begin(1, 115200, SERIAL_8N1, rxPin, txPin);      // for Arduino Nano ESP32 rxPin and txPin definition required, otherwise only slave ID and baudrate required.

  out_bool_1 = true;
  out_bool_2 = false;
  out_int = 0;
  out_float = 1.1;
}

void loop() {
  out_bool_1 = not out_bool_1;
  out_bool_2 = not out_bool_2;
  out_int -= 1;
  out_float += 1.1;
  
  modbus.poll();

  Serial.println(inp_bool_1);
  Serial.println(inp_bool_2);
  Serial.println(inp_int);
  Serial.println(inp_float);

  delay(100);
}
