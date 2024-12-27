/*
  ModbusRTUSlave32Example

  This library is a modification of ModbusRTUSlave library (https://github.com/CMB27/ModbusRTUSlave).
  The main functions and functions involving coils and discretes are unchanged.
  The functions involving holdings and inputs are modified to take in custom Modbus16bit datatype which is a combination of 4 16bit registers instead of 16bit.

  This program has been succsessfully tested with the following boards:
  - Arduino Nano

  Other components required:
  - micro USB cable

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

// setup modbus variables
bool coils[1];
bool discretes[1];
modbus32 holdings[2];
modbus32 inputs[2];

// define input output variables
bool& inp_bool = coils[0];
bool& out_bool = discretes[0];
int32_t& inp_int = holdings[0].INT32;
float& inp_float = holdings[1].FLOAT32;
int32_t& out_int = inputs[0].INT32;
float& out_float = inputs[1].FLOAT32;

// for coomunication over Arduino Nano micro USB port, dePin might be required for other applications
ModbusRTUSlave32 modbus(Serial);

void setup() {
  Serial.begin(115200);   // begin modbus serial communication

  modbus.configureCoils(coils, 1);                // bool array of coil values, number of coils
  modbus.configureDiscreteInputs(discretes, 1);   // bool array of discrete input values, number of discrete inputs
  modbus.configureHoldingRegisters(holdings, 2);  // custom 64 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputs, 2);      // custom 64 bit integer array of holding register values, number of holding registers

  modbus.begin(1, 115200);  // slave ID and baudrate

  out_bool = true;
  out_int = 0;
  out_float = 1.1;
}

void loop() {
  out_bool = not out_bool;
  out_int -= 1;
  out_float += 1.1;
  
  modbus.poll();

  delay(100);
}
