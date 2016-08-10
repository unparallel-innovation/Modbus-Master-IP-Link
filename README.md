Modbus Gateway Library for Arduino.
Brigde between a Modbus Master over TCP/IP with Modbus Slaves over Serial/UART (RS-232, RS-485) and LoRa RN-2483).
==========================================================================

A library that allows a Modbus master to communicate via IP with a Arduino that converts the messages to (RS-232, RS-485) and LoRa RN-2483

<h3>Modbus IP</h3>

There are four examples that can be accessed from the Arduino interface, once you have installed the library.
Let's look at the example Switch.ino (only the parts concerning Modbus will be commented):

```
#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModBusIPSerialLoRaLink.h>
```
Inclusion of the necessary libraries.


```
ModbusIPSerialLoRaLink modbus;
```
Create the mb instance (ModbusIP) to be used.


```
byte ip[] = { 192, 168, 1, 120 };
byte dns[] = { 8, 8, 8, 8 };
byte gateway[] = { 192, 168, 1, 1 };
byte subnet[] = { 255, 255, 255, 0 };

modbus.configIP(mac, ip, dns, gateway, subnet);
```

Sets the Ethernet shield. The values ​​of the MAC Address and the IP are passed by the config() method.
The syntax is equal to Arduino Ethernet class, and supports the following formats:

```
void config (uint8_t * mac)
void config (uint8_t * mac, IPAddress ip)
void config (uint8_t * mac, IPAddress ip, IPAddress dns)
void config (uint8_t * mac, IPAddress ip, IPAddress dns, gateway IPAddress)
void config (uint8_t * mac, IPAddress ip, IPAddress dns, IPAddress gateway, subnet IPAddress)
```

```
modbus.configSerial(&Serial1, &Serial, MODBAUD, SERIAL_8N2, MODTIMEOUT, TXENABLEPIN);
modbus.configLoRa(&Serial2, LORABAUD, LORATIMEOUT, RESETPIN);
```

This methods configure the Serial port and the LoRa radio with the following format:
```
void configSerial(HardwareSerial* SerialPort, HardwareSerial* DebugSerialPort, long baud,
        unsigned char byteFormat, long _timeout, unsigned char _TxEnablePin);

void configLoRa(HardwareSerial* LoRaPort, long baud, long _timeout, unsigned char resetpin);
```

```
modbus.update();
```

This method makes all magic, answering requests and convering messages between protocols.
It should be called only once, early in the loop.


License
=======
The code in this repo is licensed under the BSD New License. See LICENSE.txt for more info.
