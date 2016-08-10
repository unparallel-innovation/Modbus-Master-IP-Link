Modbus Master Library for Arduino to act as a Modbus Gateway (TCP/IP to Serial/UART (RS-232, RS-485) and LoRa RN-2483)
==========================================================================

A library that allows a Modbus master to communicate via IP with a Arduino that converts the messages to (RS-232, RS-485) and LoRa RN-2483

<h3>Modbus IP</h3>

There are four examples that can be accessed from the Arduino interface, once you have installed the library.
Let's look at the example Switch.ino (only the parts concerning Modbus will be commented):

```
#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModbusIP.h>
```
Inclusion of the necessary libraries.


```
ModbusIP mb;
```
Create the mb instance (ModbusIP) to be used.


```
mac byte [] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
ip byte [] = {192, 168, 1, 120};
mb.config (mac, ip);
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
