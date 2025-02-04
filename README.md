# Release Notes
*Note that version 1.1.0 has a breaking change in the software from version 1.0.0 refer to release notes below*

v 1.2.0: include function code 23 for atomic read and write where holding is written to and input is read from.
Message format from master is:
Description         size        e.g. dec    e.g. hex
Slave Address       1 byte      1           0x01
Function code       1 byte      23          0x17
Read register       2 byte      0           0x0000
Num read registers  2 byte      2           0x0002
Write register      2 byte      0           0x0000
Num write registers 2 byte      3           0x0003
Write data bytesize 1 byte      6           0x06
Data to write       x byte      1,2,3       0x000100020003
CRC                 2 byte
Message format to master follows fc 4:
Description         size        e.g. dec    e.g. hex
Slave Address       1 byte      1           0x01
Function code       1 byte      23          0x17
Read data bytesize  1 byte      4           0x04
Data to write       x byte      4,5         0x00040005
CRC                 2 byte

v 1.1.1: enable read and write of multiple 32-bit and 64-bit holding/input registers

v 1.1.0: expand modbus capability to Arduino's inbuilt USB Serial port. Code breaking change: does not begin the serial communication when running the modbus begin function. As such, the user is required to run Serial.begin(baudrate) before running modbus.begin(slave_id, baudrate). Do note that modbus.begin is still required to instantiate the slave ID and other timing parameters related to baudrate.

v1.0.0: first release. Modified from ModbusRTUSlave to add 2 additional classes: ModbusRTUSlave32 and ModbusRTUSlave64 that allows communication with datatypes that require higher number of bits (e.g. float requires 32 bits). To support reading of float or int, 2 new datatypes also created: Modbus32 and Modbus64. This new dtypes allow reading of underlying data as a float or as an int depending on the expected encoding of data.

# ModbusRTUSlave
Modbus is an industrial communication protocol. The RTU variant communicates over serial lines such as UART, RS-232, or RS-485. The full details of the Modbus protocol can be found at [modbus.org](https://modbus.org). A good summary can also be found on [Wikipedia](https://en.wikipedia.org/wiki/Modbus).

This is an Arduino library that implements the slave/server logic of the Modbus RTU protocol. This library implements function codes 1 (Read Coils), 2 (Read Discrete Inputs), 3 (Read Holding Registers), 4 (Read Input Registers), 5 (Write Single Coil), 6 (Write Single Holding Register), 15 (Write Multiple Coils), and 16 (Write Multiple Holding Registers).

This library will work with HardwareSerial, SoftwareSerial, or Serial_ (USB Serial on ATmega32u4 based boards). A driver enable pin can be set, enabling an RS-485 transceiver to be used. This library requires arrays for coils, discrete inputs, holding registers, and input registers to be passed to it.


## Future Work
To include ASCII functionality
To handle multiple encoding format (e.g. endianness) of floats and ints from master


## Compatibility
This library has been succsessfully tested with the following boards:
- Arduino Nano ESP32
- Arduino Micro

Note that ModbusRTUSlave64 uses 64bit doubles. ModbusRTUSlave64 cannot be used on Arduino boards that use 32bit doubles
(https://docs.arduino.cc/language-reference/en/variables/data-types/double/) but ModbusRTUSlave and ModbusRTUSlave32 will still work. To check the size of double, use the function sizeof().
```C++
Serial.println(sizeof(double));.
```


## Example
- [ModbusRTUSlave64Example]
- [ModbusRTUSlave32Example]
- [ModbusRTUSlaveExample]


## Methods

### ModbusRTUSlave()

#### Description
Creates a ModbusRTUSlave object and sets the serial port to use for data transmission.
Optionally sets a driver enable pin. This pin will go `HIGH` when the library is transmitting. This is primarily intended for use with an RS-485 transceiver, but it can also be a handy diagnostic when connected to an LED.

#### Syntax
``` C++
ModbusRTUSlave(serial)
ModbusRTUSlave(serial, dePin)
```

#### Parameters
- `serial`: the serial port object to use for Modbus communication.
- `dePin`: the driver enable pin. This pin is set HIGH when transmitting. If this parameter is set to `NO_DE_PIN`, this feature will be disabled. Default value is `NO_DE_PIN`. Allowed data types: `uint8_t` or `byte`.

#### Example
``` C++
# include <ModbusRTUSlaveMulti.h>

const uint8_t dePin = 13;

ModbusRTUSlave modbus(Serial, dePin);
```

---


### Modbus32

#### Description
Custom datatype for conversion of 2 uint16 registers into 1 int32 or float.
The holdingRegisters and inputRegisters array should be of this datatype.
To keep variables of int32 or float type updated, referencing can be used.

#### Syntax
``` C++
modbus32 holdingRegisters[2];
float& float_var = holdingRegister[0].FLOAT32;
uint32_t& int_var = holdingRegister[1].INT32;
```

---


### Modbus64

#### Description
Custom datatype for conversion of 4 uint16 registers into 1 int64 or double.
The holdingRegisters and inputRegisters array should be of this datatype.
To keep variables of int64 or double type updated, referencing can be used.

#### Syntax
``` C++
modbus64 holdingRegisters[2];
double& double_var = holdingRegister[0].FLOAT64;
uint64_t& long_var = holdingRegister[1].INT64;
```

---


### configureCoils()

#### Description
Tells the library where coil data is stored and the number of coils.
If this function is not run, the library will assume there are no coils.

#### Syntax
``` C++
modbus.configureCoils(coils, numCoils)
```

#### Parameters
- `coils`: an array of coil values. Allowed data types: array of `bool`.
- `numCoils`: the number of coils. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---


### configureDiscreteInputs()

#### Description
Tells the library where to read discrete input data and the number of discrete inputs.
If this function is not run, the library will assume there are no discrete inputs.

#### Syntax
``` C++
modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs)
```

#### Parameters
- `discreteInputs`: an array of discrete input values. Allowed data types: array of `bool`.
- `numDiscreteInputs`: the number of discrete inputs. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---



### configureHoldingRegisters()

#### Description
Tells the library where holding register data is stored and the number of holding registers.
If this function is not run, the library will assume there are no holding registers.

#### Syntax
``` C++
modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters)
```

#### Parameters
- `holdingRegisters`: an array of holding register values. Allowed data types: array of `Modbus64bit`.
- `numHoldingRegisters`: the number of holding registers. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---


### configureInputRegisters()

#### Description
Tells the library where to read input register data and the number of input registers.
If this function is not run, the library will assume there are no input registers.

#### Syntax
``` C++
modbus.configureInputRegisters(inputRegisters, numInputRegisters)
```

#### Parameters
- `inputRegisters`: an array of input register values. Allowed data types: array of `Modbus64bit`.
- `numInputRegisters`: the number of input registers. This value must not be larger than the size of the array. Allowed data types: `uint16_t`.

---


### begin()

#### Description
Sets the slave/server id and the data rate in bits per second (baud) for serial transmission.
Optionally it also sets the data configuration. Note, there must be 8 data bits for Modbus RTU communication. The default configuration is 8 data bits, no parity, and one stop bit.

#### Syntax
``` C++
modbus.begin(slaveId, baud)
modbus.begin(slaveId, baud, config)
```

#### Parameters
- `slaveId`: the number used to itentify this device on the Modbus network. Allowed data types: `uint8_t` or `byte`.
- `baud`: the baud rate to use for Modbus communication. Common values are: `1200`, `2400`, `4800`, `9600`, `16200`, `38400`, `57600`, and `115200`. Allowed data types: `uint32_t`.
- `config`: the serial port configuration to use. Valid values are:  
`SERIAL_8N1`: no parity (default)  
`SERIAL_8N2`  
`SERIAL_8E1`: even parity  
`SERIAL_8E2`  
`SERIAL_8O1`: odd parity  
`SERIAL_8O2`

_If using a SoftwareSerial port a configuration of `SERIAL_8N1` will be used regardless of what is entered._

---


### poll()

#### Description
Checks if any Modbus requests are available. If a valid request has been received, an appropriate response will be sent.
This function must be called frequently.

#### Syntax
``` C++
modbus.poll()
```

#### Parameters
None

#### Example
``` C++
# include <ModbusRTUSlave.h>

const uint8_t coilPins[2] = {4, 5};
const uint8_t discreteInputPins[2] = {2, 3};

ModbusRTUSlave modbus(Serial);

bool coils[2];
bool discreteInputs[2];

void setup() {
  Serial.begin(3400);
  pinMode(coilPins[0], OUTPUT);
  pinMode(coilPins[1], OUTPUT);
  pinMode(discreteInputPins[0], INPUT);
  pinMode(discreteInputPins[1], INPUT);

  modbus.configureCoils(coils, 2);
  modbus.configureDiscreteInputs(discreteInputs, 2);
  modbus.begin(1, 38400);
}

void loop() {
  discreteInputs[0] = digitalRead(discreteInputPins[0]);
  discreteInputs[1] = digitalRead(discreteInputPins[1]);

  modbus.poll();

  digitalWrite(coilPins[0], coils[0]);
  digitalWrite(coilPins[1], coils[1]);
}

```