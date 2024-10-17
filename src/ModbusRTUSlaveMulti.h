#ifndef ModbusRTUSlaveMulti
#define ModbusRTUSlaveMulti

#define MODBUS_RTU_SLAVE_BUF_SIZE 256
#define NO_DE_PIN 255
#define NO_ID 0

#include "Arduino.h"
#ifdef __AVR__
#include <SoftwareSerial.h>
#endif

// added: custom datatype to convert 2 registers of 16 bit to 32bit int/float 
// and 4 registers of 16bit to 64bit int/float
union modbus32 {
    uint16_t reg[2];
    int32_t _int;
    float _float;
    operator int32_t() const {return _int;}
    operator float() const {return _float;}
};

union modbus64 {
    uint16_t reg[4];
    int64_t _long;
    double _double;
    operator int64_t() const {return _long;}
    operator double() const {return _double;}
};
// ---

class ModbusRTUSlave {
  public:
    ModbusRTUSlave(HardwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #ifdef __AVR__
    ModbusRTUSlave(SoftwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    #ifdef HAVE_CDCSERIAL
    ModbusRTUSlave(Serial_& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    void configureCoils(bool coils[], uint16_t numCoils);
    void configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs);
    void configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters);
    void configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters);
    #ifdef ESP32
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);
    #else
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1);
    #endif
    void poll();
    
  private:
    HardwareSerial *_hardwareSerial;
    #ifdef __AVR__
    SoftwareSerial *_softwareSerial;
    #endif
    #ifdef HAVE_CDCSERIAL
    Serial_ *_usbSerial;
    #endif
    Stream *_serial;
    uint8_t _dePin;
    uint8_t _buf[MODBUS_RTU_SLAVE_BUF_SIZE];
    bool *_coils;
    bool *_discreteInputs;
    uint16_t *_holdingRegisters;
    uint16_t *_inputRegisters;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    uint8_t _id;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    #ifdef ARDUINO_ARCH_RENESAS
    unsigned long _flushCompensationDelay;
    #endif

    void _processReadCoils();
    void _processReadDiscreteInputs();
    void _processReadHoldingRegisters();
    void _processReadInputRegisters();
    void _processWriteSingleCoil();
    void _processWriteSingleHoldingRegister();
    void _processWriteMultipleCoils();
    void _processWriteMultipleHoldingRegisters();

    bool _readRequest();
    void _writeResponse(uint8_t len);
    void _exceptionResponse(uint8_t code);
    void _clearRxBuffer();

    void _calculateTimeouts(unsigned long baud, uint32_t config);
    uint16_t _crc(uint8_t len);
    uint16_t _div8RndUp(uint16_t value);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);
};

// added: ModbusRTUSlave32 and ModbusRTUSlave64
class ModbusRTUSlave32 {
  public:
    ModbusRTUSlave32(HardwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #ifdef __AVR__
    ModbusRTUSlave32(SoftwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    #ifdef HAVE_CDCSERIAL
    ModbusRTUSlave32(Serial_& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    void configureCoils(bool coils[], uint16_t numCoils);
    void configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs);
    void configureHoldingRegisters(modbus32 holdingRegisters[], uint16_t numHoldingRegisters);
    void configureInputRegisters(modbus32 inputRegisters[], uint16_t numInputRegisters);
    #ifdef ESP32
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);
    #else
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1);
    #endif
    void poll();
    
  private:
    HardwareSerial *_hardwareSerial;
    #ifdef __AVR__
    SoftwareSerial *_softwareSerial;
    #endif
    #ifdef HAVE_CDCSERIAL
    Serial_ *_usbSerial;
    #endif
    Stream *_serial;
    uint8_t _dePin;
    uint8_t _buf[MODBUS_RTU_SLAVE_BUF_SIZE];
    bool *_coils;
    bool *_discreteInputs;
    modbus32 *_holdingRegisters;
    modbus32 *_inputRegisters;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    uint8_t _id;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    #ifdef ARDUINO_ARCH_RENESAS
    unsigned long _flushCompensationDelay;
    #endif

    void _processReadCoils();
    void _processReadDiscreteInputs();
    void _processRead32bitHolding();
    void _processRead32bitInput();
    void _processWriteSingleCoil();
    void _processWriteMultipleCoils();
    void _processWrite32bitHolding();

    bool _readRequest();
    void _writeResponse(uint8_t len);
    void _exceptionResponse(uint8_t code);
    void _clearRxBuffer();

    void _calculateTimeouts(unsigned long baud, uint32_t config);
    uint16_t _crc(uint8_t len);
    uint16_t _div8RndUp(uint16_t value);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);
};

class ModbusRTUSlave64 {
  public:
    ModbusRTUSlave64(HardwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #ifdef __AVR__
    ModbusRTUSlave64(SoftwareSerial& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    #ifdef HAVE_CDCSERIAL
    ModbusRTUSlave64(Serial_& serial, uint8_t dePin = NO_DE_PIN);
    #endif
    void configureCoils(bool coils[], uint16_t numCoils);
    void configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs);
    void configureHoldingRegisters(modbus64 holdingRegisters[], uint16_t numHoldingRegisters);
    void configureInputRegisters(modbus64 inputRegisters[], uint16_t numInputRegisters);
    #ifdef ESP32
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);
    #else
    void begin(uint8_t id, unsigned long baud, uint32_t config = SERIAL_8N1);
    #endif
    void poll();
    
  private:
    HardwareSerial *_hardwareSerial;
    #ifdef __AVR__
    SoftwareSerial *_softwareSerial;
    #endif
    #ifdef HAVE_CDCSERIAL
    Serial_ *_usbSerial;
    #endif
    Stream *_serial;
    uint8_t _dePin;
    uint8_t _buf[MODBUS_RTU_SLAVE_BUF_SIZE];
    bool *_coils;
    bool *_discreteInputs;
    modbus64 *_holdingRegisters;
    modbus64 *_inputRegisters;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    uint8_t _id;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    #ifdef ARDUINO_ARCH_RENESAS
    unsigned long _flushCompensationDelay;
    #endif

    void _processReadCoils();
    void _processReadDiscreteInputs();
    void _processRead64bitHolding();
    void _processRead64bitInput();
    void _processWriteSingleCoil();
    void _processWriteMultipleCoils();
    void _processWrite64bitHolding();

    bool _readRequest();
    void _writeResponse(uint8_t len);
    void _exceptionResponse(uint8_t code);
    void _clearRxBuffer();

    void _calculateTimeouts(unsigned long baud, uint32_t config);
    uint16_t _crc(uint8_t len);
    uint16_t _div8RndUp(uint16_t value);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);
};

#endif
