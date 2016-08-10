#include <SPI.h>
#include <Ethernet2.h>
#include <Modbus.h>

#include <ModBusIPSerialLink.h>

//////////////////// Port information ///////////////////
#define BAUD 57600
#define TIMEOUT 2000
#define TXENABLEPIN A9

//ModbusIP object
ModbusIPSerial modbus;

unsigned long ts;

void setup() {
  Serial.begin(57600);
  Serial.println("Hello! - ModBus TCP/IP <-> Serial/UART Link");

  // Initialize the Modbus Finite State Machine
  modbus.configSerial(&Serial1, &Serial, BAUD, SERIAL_8N2, TIMEOUT, TXENABLEPIN);

  // The media access control (ethernet hardware) address for the shield
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

  // The IP address for the shield
  byte ip[] = { 172, 20, 254, 2 };
  byte dns[] = { 8, 8, 8, 8 };
  byte gateway[] = { 172, 20, 254, 1 };
  byte subnet[] = { 255, 255, 255, 0 };

  //Config Modbus IP
  modbus.configIP(mac, ip, dns, gateway, subnet);
}

void loop() {
  modbus.update();
}
