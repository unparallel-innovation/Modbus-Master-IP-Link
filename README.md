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
const int SWITCH_ISTS = 100;
```
Sets the Modbus register to represent the switch. This value is the offset (0-based) to be placed in its supervisory or testing software.
Note that if your software uses offsets 1-based the set value there should be 101, for this example.

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

Then we have:
```
mb.addIsts (SWITCH_ISTS);
```
Adds the register type Input Status (digital input) that is responsible for detecting if a switch is in state on or off.
The library allows you to set an initial value for the register:

```
mb.addIsts (SWITCH_ISTS, true);
```
In this case the register is added and set to true. If you use the first form the default value is false.


```
mb.task ();
```
This method makes all magic, answering requests and changing the registers if necessary, it should be called only once, early in the loop.


```
mb.Ists (SWITCH_ISTS, digitalRead (switchPin));
```
Finally the value of SWITCH_ISTS register changes as the state of the selected digital input.
