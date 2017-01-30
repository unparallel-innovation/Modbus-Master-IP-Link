#include <SPI.h>
#include <Ethernet2.h>
#include <Modbus.h>

#include <ModBusIPSerialLink.h>

//////////////////// Port information ///////////////////
#define BAUD 9600
#define TIMEOUT 2000
#define TXENABLEPIN A8

//ModbusIP object
ModbusIPSerial modbus;

void setup() {
  Serial.begin(9600);
  Serial.println("Hello! - ModBus TCP/IP <-> Serial/UART Gateway");

  // Initialize the Modbus Finite State Machine
  modbus.configSerial(&Serial1, &Serial, BAUD, SERIAL_8N1, TIMEOUT, TXENABLEPIN);

  // The media access control (ethernet hardware) address for the shield
  byte mac[] = { 0xDD, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
  
  byte ip[] = { 172, 20, 254, 2 };

  //Config Modbus IP
  modbus.configIP(mac, ip);
}

void loop() {
  modbus.update();
}
