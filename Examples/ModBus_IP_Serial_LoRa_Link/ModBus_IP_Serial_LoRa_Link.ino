#include <SPI.h>
#include <Ethernet2.h>
#include <Modbus.h>

#include <ModBusIPSerialLoRaLink.h>

//////////////////// ModBus Port information ///////////////////
#define MODBAUD 57600
#define MODTIMEOUT 3000
#define TXENABLEPIN A9

//////////////////// LoRa Port information ///////////////////
#define LORABAUD 57600
#define LORATIMEOUT 2500
#define RESETPIN A0

//ModbusIP object
ModbusIPSerialLoRaLink modbus;

unsigned long ts;

void setup() {
  Serial.begin(57600);
  Serial.println("Hello! - ModBus TCP/IP <-> Serial/UART Link");
  Serial.println();

  // Initialize the Modbus Finite State Machine
  modbus.configSerial(&Serial1, &Serial, MODBAUD, SERIAL_8N2, MODTIMEOUT, TXENABLEPIN);

  modbus.configLoRa(&Serial2, LORABAUD, LORATIMEOUT, RESETPIN);

  // The media access control (ethernet hardware) address for the shield
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

  // The IP address for the shield
  byte ip[] = { 172, 20, 254, 2 };

  //Config Modbus IP
  modbus.configIP(mac, ip);
}

void loop() {
  modbus.update();
}
