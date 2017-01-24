//#include <SPI.h>
//#include <Ethernet2.h>
//#include <Modbus.h>

#include <ModBusGateway_RS485_RFM95_to_TCP_IP.h>

//////////////////// ModBus Port information ///////////////////
#define MODBUS_BAUD 57600
#define MODBUS_TIMEOUT 5000
#define TX_ENABLE_PIN A8

#define RESETPIN A12

#define SERVER_ADDRESS 250

#define RF95_FREQ 869.0

// Singleton instance of the radio driver
RH_RF95 driver(A10, 20);

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
  modbus.configSerial(&Serial1, &Serial, MODBUS_BAUD, SERIAL_8N2, MODBUS_TIMEOUT, TX_ENABLE_PIN);

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
  driver.setTxPower(23, false);

  // Configure the radio for long range
  // < Bw = 125 kHz, Cr = 4/5, Sf = 4096chips/symbol, CRC on. Slower + longer range
  //  RH_RF95::ModemConfig RFM95config = { 0x72,   0xc4,    0x00}; // Bw125Cr45Sf4096
  // < Bw = 125 kHz, Cr = 4/5, Sf = 512chips/symbol, CRC on. Slow + long range
  RH_RF95::ModemConfig RFM95config = { 0x72,   0x94,    0x00};  // Bw125Cr45Sf512
  driver.setModemRegisters(&RFM95config);

  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
  //  driver.setCADTimeout(10000);

  // Set the frequency of the RFM95
  if (!driver.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }
}

void loop() {
  modbus.update();
}
