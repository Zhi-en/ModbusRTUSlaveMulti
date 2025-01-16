// created: modify functions relating to holding and input registers for 32bit protocol using 2 registers at a time
// combined HardwareSerial, SofrwareSerial and Serial_ types into Stream type, removed Serial begin in modbus begin
#include "ModbusRTUSlaveMulti.h"

ModbusRTUSlave32::ModbusRTUSlave32(Stream& serial, uint8_t dePin) {
  _serial = &serial;
  _coils = 0;
  _discreteInputs = 0;
  _holdingRegisters = 0;
  _inputRegisters = 0;
  _dePin = dePin;
}

void ModbusRTUSlave32::configureCoils(bool coils[], uint16_t numCoils) {
  _coils = coils;
  _numCoils = numCoils;
}

void ModbusRTUSlave32::configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs) {
  _discreteInputs = discreteInputs;
  _numDiscreteInputs = numDiscreteInputs;
}

void ModbusRTUSlave32::configureHoldingRegisters(modbus32 holdingRegisters[], uint16_t numHoldingRegisters) {
  _holdingRegisters = holdingRegisters;
  _numHoldingRegisters = numHoldingRegisters*2;
}

void ModbusRTUSlave32::configureInputRegisters(modbus32 inputRegisters[], uint16_t numInputRegisters) {
  _inputRegisters = inputRegisters;
  _numInputRegisters = numInputRegisters*2;
}

void ModbusRTUSlave32::begin(uint8_t id, unsigned long baud, uint32_t config) {
  if (id >= 1 && id <= 247) _id = id;
  else _id = NO_ID;
  _calculateTimeouts(baud, config);
  if (_dePin != NO_DE_PIN) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  _clearRxBuffer();
}

void ModbusRTUSlave32::poll() {
  if (_serial->available()) {
    if (_readRequest()) {
      switch (_buf[1]) {
        case 1:
          _processReadCoils();
          break;
        case 2:
          _processReadDiscreteInputs();
          break;
        case 3:
          _processRead32bitHoldings();
          break;
        case 4:
          _processRead32bitInputs();
          break;
        case 5:
          _processWriteSingleCoil();
          break;
        case 6:
          _exceptionResponse(4);  // 32-bit protocol should not use single holding write function
          break;
        case 15:
          _processWriteMultipleCoils();
          break;
        case 16:
          _processWrite32bitHoldings();
          break;
        default:
          _exceptionResponse(1);
          break;
      }
    }
  }
}

void ModbusRTUSlave32::_processReadCoils() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) _exceptionResponse(2);
  else {
    _buf[2] = _div8RndUp(quantity);
    for (uint16_t i = 0; i < quantity; i++) {
      bitWrite(_buf[3 + (i >> 3)], i & 7, _coils[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave32::_processReadDiscreteInputs() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_discreteInputs || _numDiscreteInputs == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) _exceptionResponse(3);
  else if (quantity > _numDiscreteInputs || startAddress > (_numDiscreteInputs - quantity)) _exceptionResponse(2);
  else {
    _buf[2] = _div8RndUp(quantity);
    for (uint16_t i = 0; i < quantity; i++) {
      bitWrite(_buf[3 + (i >> 3)], i & 7, _discreteInputs[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave32::_processRead32bitHoldings() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125) _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) _exceptionResponse(2);
  else if (quantity % 2 != 0 || startAddress % 2 != 0) _exceptionResponse(4);
  else {
    _buf[2] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_holdingRegisters[(startAddress + i)/ 2].reg[1 - (i % 2)]);  // Arduino uses little endian while most devices use big endian: flip sequence of array
      _buf[4 + (i * 2)] = lowByte(_holdingRegisters[(startAddress + i)/ 2].reg[1 - (i % 2)]);   // integer division gives floor results (startAddress + i)/4 is the same as floor((startAddess + i)/4.0)
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave32::_processRead32bitInputs() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_inputRegisters || _numInputRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125) _exceptionResponse(3);
  else if (quantity > _numInputRegisters || startAddress > (_numInputRegisters - quantity)) _exceptionResponse(2);
  else if (quantity % 2 != 0 || startAddress % 2 != 0) _exceptionResponse(4);
  else {
    _buf[2] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++) {
      _buf[3 + (i * 2)] = highByte(_inputRegisters[(startAddress + i)/ 2].reg[1 - (i % 2)]);    // Arduino uses little endian while most devices use big endian: flip sequence of array
      _buf[4 + (i * 2)] = lowByte(_inputRegisters[(startAddress + i)/ 2].reg[1 - (i % 2)]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave32::_processWriteSingleCoil() {
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils ||_numCoils == 0) _exceptionResponse(1);
  else if (value != 0 && value != 0xFF00) _exceptionResponse(3);
  else if (address >= _numCoils) _exceptionResponse(2);
  else {
    _coils[address] = value;
    _writeResponse(6);
  }
}

void ModbusRTUSlave32::_processWriteMultipleCoils() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 1968 || _buf[6] != _div8RndUp(quantity)) _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) _exceptionResponse(2);
  else {
    for (uint16_t i = 0; i < quantity; i++) {
      _coils[startAddress + i] = bitRead(_buf[7 + (i >> 3)], i & 7);
    }
    _writeResponse(6);
  }
}

void ModbusRTUSlave32::_processWrite32bitHoldings() {
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _exceptionResponse(1);
  else if (quantity == 0 || quantity > 123 || _buf[6] != (quantity * 2)) _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) _exceptionResponse(2);
  else if (quantity % 2 != 0 || startAddress % 2 != 0) _exceptionResponse(4);
  else {
    for (uint16_t i = 0; i < quantity; i++) {
      _holdingRegisters[(startAddress + i)/ 2].reg[1 - (i % 2)] = _bytesToWord(_buf[i * 2 + 7], _buf[i * 2 + 8]);   // Arduino uses little endian while most devices use big endian: flip sequence of array
    }
    _writeResponse(6);
  }
}

bool ModbusRTUSlave32::_readRequest() {
  uint16_t numBytes = 0;
  unsigned long startTime = 0;
  do {
    if (_serial->available()) {
      startTime = micros();
      _buf[numBytes] = _serial->read();
      numBytes++;
    }
  } while (micros() - startTime <= _charTimeout && numBytes < MODBUS_RTU_SLAVE_BUF_SIZE);
  while (micros() - startTime < _frameTimeout);
  if (!_serial->available() && (_buf[0] == _id || _buf[0] == 0) && _crc(numBytes - 2) == _bytesToWord(_buf[numBytes - 1], _buf[numBytes - 2])) return true;
  else return false;
}

void ModbusRTUSlave32::_writeResponse(uint8_t len) {
  if (_buf[0] != 0) {
    uint16_t crc = _crc(len);
    _buf[len] = lowByte(crc);
    _buf[len + 1] = highByte(crc);
    if (_dePin != NO_DE_PIN) digitalWrite(_dePin, HIGH);
    _serial->write(_buf, len + 2);
    _serial->flush();
    #ifdef ARDUINO_ARCH_RENESAS
    delayMicroseconds(_flushCompensationDelay);
    #endif
    if (_dePin != NO_DE_PIN) digitalWrite(_dePin, LOW);
    while(_serial->available()) {
      _serial->read();
    }
  }
}

// code 1: unable to process request
// code 2: registers requested exceed registers instantiated
// code 3: register number exceeds register limit of 0 to 125
// code 4: write to holding / read from input request does not follow 32-bit (2 register) format (added)
void ModbusRTUSlave32::_exceptionResponse(uint8_t code) {
  _buf[1] |= 0x80;
  _buf[2] = code;
  _writeResponse(3);
}

void ModbusRTUSlave32::_clearRxBuffer() {
  unsigned long startTime = micros();
  do {
    if (_serial->available()) {
      startTime = micros();
      _serial->read();
    }
  } while (micros() - startTime < _frameTimeout);
}



void ModbusRTUSlave32::_calculateTimeouts(unsigned long baud, uint32_t config) {
  unsigned long bitsPerChar;
  if (config == SERIAL_8E2 || config == SERIAL_8O2) bitsPerChar = 12;
  else if (config == SERIAL_8N2 || config == SERIAL_8E1 || config == SERIAL_8O1) bitsPerChar = 11;
  else bitsPerChar = 10;
  if (baud <= 19200) {
    _charTimeout = (bitsPerChar * 2500000) / baud;
    _frameTimeout = (bitsPerChar * 4500000) / baud;
  }
  else {
    _charTimeout = (bitsPerChar * 1000000) / baud + 750;
    _frameTimeout = (bitsPerChar * 1000000) / baud + 1750;
  }
  #ifdef ARDUINO_ARCH_RENESAS
  _flushCompensationDelay = (bitsPerChar * 1000000) / baud;
  #endif
}

uint16_t ModbusRTUSlave32::_crc(uint8_t len) {
  uint16_t value = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    value ^= (uint16_t)_buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb) value ^= 0xA001;
    }
  }
  return value;
}

uint16_t ModbusRTUSlave32::_div8RndUp(uint16_t value) {
  return (value + 7) >> 3;
}

uint16_t ModbusRTUSlave32::_bytesToWord(uint8_t high, uint8_t low) {
  return (high << 8) | low;
}
