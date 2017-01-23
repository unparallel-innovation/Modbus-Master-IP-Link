//#include <SPI.h>
//#include <Ethernet2.h>
//#include <Modbus.h>

#include <ModBusGateway_RS485_RFM95_to_TCP_IP.h>

//////////////////// ModBus Port information ///////////////////
#define MODBAUD 57600
#define MODTIMEOUT 5000
#define TXENABLEPIN A8

#define RESETPIN A12

#define SERVER_ADDRESS 250

#define RF95_FREQ 868.5

// Singleton instance of the radio driver
RH_RF95 driver(A10, 20);
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

//ModbusIP object
ModBusGateway_RS485_RFM95_to_TCP_IP modbus(manager);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

unsigned long ts;

void setup() {
  pinMode(4, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Hello! - ModBus TCP/IP <-> Serial/UART Link");
  Serial.println();

  if (!manager.init())
    Serial.println("init failed");

  // Initialize the Modbus Finite State Machine
  modbus.configSerial(&Serial1, &Serial, MODBAUD, SERIAL_8N2, MODTIMEOUT, TXENABLEPIN);

  modbus.configLoRa(RESETPIN, buf, sizeof(buf));

  // The media access control (ethernet hardware) address for the shield
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

  // The IP address for the shield
  byte ip[] = { 172, 20, 254, 2 };

  //Config Modbus IP
  modbus.configIP(mac, ip);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //  driver.setTxPower(23, false);
  driver.setTxPower(15, false);

  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
  //  driver.setCADTimeout(10000);

  // You can optionally set the frequency of the RFM95
  //  if (!driver.setFrequency(RF95_FREQ)) {
  //    Serial.println("setFrequency failed");
  //  }
}

void loop() {
  modbus.update();
}
