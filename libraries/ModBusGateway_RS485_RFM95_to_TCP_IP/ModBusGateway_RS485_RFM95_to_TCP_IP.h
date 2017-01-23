#ifndef MODBUSGATEWAY_RS485_RFM95_TO_TCP_IP_H
#define MODBUSGATEWAY_RS485_RFM95_TO_TCP_IP_H

/*
   Note:
   Using a USB to Serial converter the maximum bytes you can send is
   limited to its internal buffer which differs between manufactures.

   Since it is assumed that you will mostly use the Arduino to connect without
   using a USB to Serial converter the internal buffer is set the same as the
   Arduino Serial ring buffer which is 64 bytes.
*/

#include "Arduino.h"
#include <Modbus.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define TCP_KEEP_ALIVE

#define COIL_OFF 0x0000 // Function 5 OFF request is 0x0000
#define COIL_ON 0xFF00 // Function 5 ON request is 0xFF00
#define READ_COIL_STATUS 1 // Reads the ON/OFF status of discrete outputs (0X references, coils) in the slave.
#define READ_INPUT_STATUS 2 // Reads the ON/OFF status of discrete inputs (1X references) in the slave.
#define READ_HOLDING_REGISTERS 3 // Reads the binary contents of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS 4 // Reads the binary contents of input registers (3X references) in the slave. Not writable.
#define FORCE_SINGLE_COIL 5 // Forces a single coil (0X reference) to either ON (0xFF00) or OFF (0x0000).
#define PRESET_SINGLE_REGISTER 6 // Presets a value into a single holding register (4X reference).
#define FORCE_MULTIPLE_COILS 15 // Forces each coil (0X reference) in a sequence of coils to either ON or OFF.
#define PRESET_MULTIPLE_REGISTERS 16 // Presets values into a sequence of holding registers (4X references).

#define MODBUSIP_PORT 	  502
#define MODBUSIP_MAXFRAME 200

// state machine states
#define IDLE 1
#define WAITING_FOR_REPLY 2

#define BUFFER_SIZE RH_RF95_MAX_MESSAGE_LEN

typedef struct
{
  // specific packet info
  unsigned char id;
  unsigned char function;
  unsigned int  address;

	// For functions 1 & 2 data is the number of points
	// For function 5 data is either ON (oxFF00) or OFF (0x0000)
	// For function 6 data is exactly that, one register's data
  // For functions 3, 4 & 16 data is the number of registers
  // For function 15 data is the number of coils
  unsigned int data;

	unsigned int local_start_address;

  // modbus information counters
  unsigned int requests;
  unsigned int successful_requests;
	unsigned int failed_requests;
	unsigned int exception_errors;
  unsigned int retries;

  // connection status of packet
  unsigned char connection;
}Packet;

class ModBusGateway_RS485_RFM95_to_TCP_IP : public Modbus {
    private:
        // Ethernet Variables
        EthernetServer _server;
        EthernetClient _client;
        byte _MBAP[7];

        // Modbus Variables
        HardwareSerial* ModbusPort;
        unsigned char TxEnablePin;

        // frame[] is used to receive and transmit packages.
        // The maximum number of bytes in a modbus packet is 256 bytes
        // This is limited to the serial buffer of 64 bytes
        unsigned char frame[BUFFER_SIZE];
        unsigned char buffer_len;  // variable to store buffer size
        long timeout; // timeout interval
        unsigned int T1_5; // inter character time out in microseconds
        unsigned int frameDelay; // frame time out in microseconds
        unsigned long delayStart; // init variable for turnaround and timeout delay
        unsigned char state;

        Packet * packet; // current packet

        HardwareSerial* DebugPort;

        // LoRa Variables
        RHReliableDatagram &_manager;
        unsigned char resetPin;
        uint8_t * buff;
        int size_of_buff;
        uint8_t rf95_len;

        // function definitions
        void task();
        void idle();
        void waiting_for_reply();
        void processReply();
        void processError();
        void processSuccess();
        unsigned int calculateCRC(unsigned char * frameIn, unsigned char bufferSize);
        char hexToC(char c);
        char hexToChar(char c1, char c2);

    public:
        ModBusGateway_RS485_RFM95_to_TCP_IP(RHReliableDatagram & manager);

        void update();

        void configIP(uint8_t *mac);
        void configIP(uint8_t *mac, IPAddress ip);
        void configIP(uint8_t *mac, IPAddress ip, IPAddress dns);
        void configIP(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway);
        void configIP(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet);

        void configSerial(HardwareSerial* SerialPort,
        	                 HardwareSerial* DebugSerialPort,
        	                 long baud,
        	                 unsigned char byteFormat,
        	                 long _timeout,
        	                 unsigned char _TxEnablePin);

        void configLoRa(unsigned char resetpin, uint8_t * buf, int size_of_buf);
};

#endif   // MODBUSGATEWAY_RS485_RFM95_TO_TCP_IP_H
