#include "ModBusIPSerialLoRaLink.h"
#include "HardwareSerial.h"

ModbusIPSerialLoRaLink::ModbusIPSerialLoRaLink():_server(MODBUSIP_PORT) {
}

void ModbusIPSerialLoRaLink::configIP(uint8_t *mac) {
	Ethernet.begin(mac);
	_server.begin();
}

void ModbusIPSerialLoRaLink::configIP(uint8_t *mac, IPAddress ip) {
	Ethernet.begin(mac, ip);
	_server.begin();
}

void ModbusIPSerialLoRaLink::configIP(uint8_t *mac, IPAddress ip, IPAddress dns) {
	Ethernet.begin(mac, ip, dns);
	_server.begin();
}

void ModbusIPSerialLoRaLink::configIP(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway) {
	Ethernet.begin(mac, ip, dns, gateway);
	_server.begin();
}

void ModbusIPSerialLoRaLink::configIP(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet) {
	Ethernet.begin(mac, ip, dns, gateway, subnet);
	_server.begin();
}

void ModbusIPSerialLoRaLink::configSerial(HardwareSerial* SerialPort,
	HardwareSerial* DebugSerialPort,
	long baud,
	unsigned char byteFormat,
	long _timeout,
	unsigned char _TxEnablePin)
	{
		// Modbus states that a baud rate higher than 19200 must use a fixed 750 us
		// for inter character time out and 1.75 ms for a frame delay for baud rates
		// below 19200 the timing is more critical and has to be calculated.
		// E.g. 9600 baud in a 11 bit packet is 9600/11 = 872 characters per second
		// In milliseconds this will be 872 characters per 1000ms. So for 1 character
		// 1000ms/872 characters is 1.14583ms per character and finally modbus states
		// an inter-character must be 1.5T or 1.5 times longer than a character. Thus
		// 1.5T = 1.14583ms * 1.5 = 1.71875ms. A frame delay is 3.5T.
		// Thus the formula is T1.5(us) = (1000ms * 1000(us) * 1.5 * 11bits)/baud
		// 1000ms * 1000(us) * 1.5 * 11bits = 16500000 can be calculated as a constant

		if (baud > 19200)
		T1_5 = 750;
		else
		T1_5 = 16500000/baud; // 1T * 1.5 = T1.5

		/* The modbus definition of a frame delay is a waiting period of 3.5 character times
		between packets. This is not quite the same as the frameDelay implemented in
		this library but does benifit from it.
		The frameDelay variable is mainly used to ensure that the last character is
		transmitted without truncation. A value of 2 character times is chosen which
		should suffice without holding the bus line high for too long.*/

		frameDelay = T1_5 * 3.5;

		// initialize
		_frame = frame;
		state = IDLE;
		timeout = _timeout;
		TxEnablePin = _TxEnablePin;
		packet = new Packet();

		ModbusPort = SerialPort;
		DebugPort = DebugSerialPort;
		(*ModbusPort).begin(baud, byteFormat);
		pinMode(TxEnablePin, OUTPUT);
		digitalWrite(TxEnablePin, LOW);
	}

	void ModbusIPSerialLoRaLink::configLoRa(HardwareSerial* LoRaPort,
		long baud,
		long _timeout,
		unsigned char resetpin)
		{
			// initialize
			_frame = frame;
			state = IDLE;
			timeout_lora = _timeout;
			resetPin = resetpin;
			lastTxMillis = 0;

			LoraPort = LoRaPort;

			pinMode(resetPin, OUTPUT);
			digitalWrite(resetPin, LOW);
			delay(100);
			pinMode(resetPin, INPUT);
			digitalWrite(resetPin, INPUT_PULLUP);

			(*LoraPort).begin(baud);
			(*LoraPort).setTimeout(timeout_lora);

			unsigned long currentMillis = millis();
			(*LoraPort).flush();
			delayMicroseconds(frameDelay);
			while (!(*LoraPort).available()  && millis() - currentMillis < timeout_lora);
			String str = (*LoraPort).readStringUntil('\n');

			send_cmd_lora("sys reset", true, true, false);
			send_cmd_lora("mac pause", true, true, false);

			char buf[20];
			char rtx[] = "radio set wdt ";
			char c[6];

			for (byte i = 0; i < 14; ++i) {
				buf[i] = rtx[i];
			}

			int n = log10(timeout_lora) + 1;
			if(n > 6) n = 6;

			sprintf(c, "%ld", timeout_lora);

			for (int i = 0; i < n; ++i)
			{
				buf[14+i] = c[i];
			}

			buf[14+n] = '\0';

			send_cmd_lora(buf, true, true, false);
			(*DebugPort).println(F("-----------"));
		}

		void ModbusIPSerialLoRaLink::task() {
			EthernetClient client = _server.available();
			_client = client;

			if (client) {
				if (client.connected()) {
					(*DebugPort).println();
					(*DebugPort).println(F("-----------------------------------------------------------------------------------------------"));
					(*DebugPort).println();

					int i = 0;
					while (client.available()){
						_MBAP[i] = client.read();
						i++;
						if (i==7) break;  //MBAP length has 7 bytes size
					}
					_len = _MBAP[4] << 8 | _MBAP[5];
					_len--;  // Do not count with last byte from MBAP

					if (_MBAP[2] !=0 || _MBAP[3] !=0) return;   //Not a MODBUSIP packet
					if (_len > MODBUSIP_MAXFRAME) return;      //Length is over MODBUSIP_MAXFRAME

					(*DebugPort).println(F("GOT IP MBAP"));
					for (i=0 ; i < 7 ; i++) {
						(*DebugPort).print(_MBAP[i]);
						(*DebugPort).print(":");
						//(*DebugPort).println(_frame[i], HEX);
					}
					(*DebugPort).println();
					(*DebugPort).println(F("-----------------"));

					_frame[0] = _MBAP[6]; // Slave ID
					packet->id = _frame[0];
					i = 1;
					_len += 1;
					while (client.available()){
						_frame[i] = client.read();
						i++;
						if (i==_len) break;
					}

					(*DebugPort).println(F("GOT IP PDU"));
					for (i=1 ; i < _len ; i++) {
						(*DebugPort).print(_frame[i]);
						(*DebugPort).print(":");
						//(*DebugPort).println(_frame[i], HEX);
					}
					(*DebugPort).println();
					(*DebugPort).println(F("-----------------"));

					unsigned int crc16 = calculateCRC(_frame, _len);
					_len += 2;
					_frame[_len - 2] = crc16 >> 8; // split crc into 2 bytes
					_frame[_len - 1] = crc16 & 0xFF;

					(*DebugPort).println(F("SENT Serial CMD"));
					for (unsigned char i = 0; i < _len; i++) {
						(*DebugPort).print(_frame[i], DEC);
						(*DebugPort).print(":");
					}
					(*DebugPort).println();
					(*DebugPort).println(F("-----------------"));

					digitalWrite(TxEnablePin, HIGH);

					for (unsigned char i = 0; i < _len; i++) {
						(*ModbusPort).write(_frame[i]);
					}
					(*ModbusPort).flush();

					delayMicroseconds(frameDelay);

					digitalWrite(TxEnablePin, LOW);

					char buf[10 + _len*2];
					char rtx[] = "radio tx ";
					char c2[2];

					for (byte i = 0; i < 9; ++i) {
						buf[i] = rtx[i];
					}

					for (int i = 0; i < _len; ++i)
					{
						sprintf(c2, "%02x", _frame[i]);
						buf[9+2*i] = c2[0];
						buf[10+2*i] = c2[1];
					}
					buf[9+2*_len] = '\0';

					while (millis() - lastTxMillis < timeout_lora);

					this->send_cmd_lora(buf, true, true, true);
					lastTxMillis = millis();
					this->send_cmd_lora("radio rx 0", false, false, false);

					delayStart = millis(); // start the timeout delay
					state = WAITING_FOR_REPLY;

					// #ifndef TCP_KEEP_ALIVE
					// client.stop();
					// #endif

					_len = 0;
				}
			}
		}

		// Modbus Master State Machine
		void ModbusIPSerialLoRaLink::update()
		{
			switch (state)
			{
				case IDLE:
				this->task();
				break;
				case WAITING_FOR_REPLY:
				this->waiting_for_reply();
				break;
			}
		}

		// get the serial data from the buffer
		void ModbusIPSerialLoRaLink::waiting_for_reply()
		{
			if ((*LoraPort).available()) // is there something to check?
			{
				unsigned char overflowFlag = 0;
				while ((*LoraPort).available() > buffer_len)	{
					buffer_len = (*LoraPort).available();
					delayMicroseconds(T1_5);

					if (buffer_len >= BUFFER_SIZE)
					overflowFlag = 1;
				}

				if (buffer_len < 10) return;

				String str = (*LoraPort).readStringUntil('\n');
				(*DebugPort).println(F("GOT LoRa Message"));
				(*DebugPort).println(str);
				(*DebugPort).println(F("-----------------"));

				String recv_msg = "radio_rx  ";

				int foundpos = -1;
				for (int i = 0; i <= str.length() - recv_msg.length(); i++) {
					if (str.substring(i,recv_msg.length()+i) == recv_msg) {
						foundpos = i;
					}
				}

				if( foundpos == 0)
				{
					buffer_len = ((str.length()-11)/2);

					byte frame_aux[str.length()];
					str.getBytes(frame_aux, str.length());

					(*DebugPort).println(F("GOT LoRa RESPONSE"));
					for (int i = 0; i < buffer_len; ++i) {
						frame[i] = hexToChar(frame_aux[2*i+10], frame_aux[2*i+11]);
						(*DebugPort).print(frame[i]);
						(*DebugPort).print(":");
					}
					(*DebugPort).println();
					(*DebugPort).println(F("-----------------"));

					// The minimum buffer size from a slave can be an exception response of
					// 5 bytes. If the buffer was partially filled set a frame_error.
					// The maximum number of bytes in a modbus packet is 256 bytes.
					// The serial buffer limits this to 64 bytes.
					if ((buffer_len < 5) || overflowFlag) {
						(*DebugPort).println(F("-------- ERROR Overflow Flag --------"));
						this->processError();
					}

					// Modbus over serial line datasheet states that if an unexpected slave
					// responded the master must do nothing and continue with the time out.
					// This seems silly cause if an incorrect slave responded you would want to
					// have a quick turnaround and poll the right one again. If an unexpected
					// slave responded it will most likely be a frame error in any event
					else if (frame[0] != packet->id) { // check id returned
						(*DebugPort).println(F("-------- ERROR Wrong ID --------"));
						this->processError();
					}
					else
					this->processReply();
				}
			}
			if ((*ModbusPort).available()) // is there something to check?
			{
				unsigned char overflowFlag = 0;
				buffer_len = 0;
				while ((*ModbusPort).available())
				{
					// The maximum number of bytes is limited to the serial buffer size
					// of BUFFER_SIZE. If more bytes is received than the BUFFER_SIZE the
					// overflow flag will be set and the serial buffer will be read until
					// all the data is cleared from the receive buffer, while the slave is
					// still responding.
					if (overflowFlag)
					(*ModbusPort).read();
					else
					{
						if (buffer_len == BUFFER_SIZE)
						overflowFlag = 1;

						frame[buffer_len] = (*ModbusPort).read();
						buffer_len++;
					}
					// This is not 100% correct but it will suffice.
					// worst case scenario is if more than one character time expires
					// while reading from the buffer then the buffer is most likely empty
					// If there are more bytes after such a delay it is not supposed to
					// be received and thus will force a frame_error.
					delayMicroseconds(T1_5); // inter character time out
				}

				(*DebugPort).println(F("GOT Serial RESPONSE"));
				for (int i=0 ; i < buffer_len ; i++) {
					(*DebugPort).print(frame[i], DEC);
					(*DebugPort).print(":");
					//(*DebugPort).println(sendbuffer[i], HEX);
				}
				(*DebugPort).println();
				(*DebugPort).println(F("-----------------"));

				// The minimum buffer size from a slave can be an exception response of
				// 5 bytes. If the buffer was partially filled set a frame_error.
				// The maximum number of bytes in a modbus packet is 256 bytes.
				// The serial buffer limits this to 64 bytes.
				if ((buffer_len < 5) || overflowFlag) {
					(*DebugPort).println(F("-------- ERROR Overflow Flag --------"));
					this->processError();
				}

				// Modbus over serial line datasheet states that if an unexpected slave
				// responded the master must do nothing and continue with the time out.
				// This seems silly cause if an incorrect slave responded you would want to
				// have a quick turnaround and poll the right one again. If an unexpected
				// slave responded it will most likely be a frame error in any event
				else if (frame[0] != packet->id) { // check id returned
					(*DebugPort).println(F("-------- ERROR Wrong ID --------"));
					this->processError();
				}
				else {
					this->processReply();
				}
				
				String str = (*LoraPort).readStringUntil('\n');

			}
			else if ((millis() - delayStart) > timeout) // check timeout
			{
				(*DebugPort).println(F("-------- ERROR Timeout --------"));
				this->processError();
				state = IDLE; //state change, override processError() state
			}
			buffer_len = 0;
		}

		void ModbusIPSerialLoRaLink::processReply()
		{
			// combine the crc Low & High bytes
			unsigned int received_crc = ((frame[buffer_len - 2] << 8) | frame[buffer_len - 1]);
			unsigned int calculated_crc = this->calculateCRC(frame, buffer_len - 2);

			if (calculated_crc == received_crc) // verify checksum
			{
				// To indicate an exception response a slave will 'OR'
				// the requested function with 0x80
				if ((frame[1] & 0x80) == 0x80) // extract 0x80
				{
					(*DebugPort).println(F("-------- ERROR 0x80 --------"));
					packet->exception_errors++;
					this->processError();
				}
				else
				{
					switch (frame[1]) // check function returned
					{
						case READ_COIL_STATUS:
						case READ_INPUT_STATUS:
						case READ_INPUT_REGISTERS:
						case READ_HOLDING_REGISTERS:
						case FORCE_SINGLE_COIL:
						case PRESET_SINGLE_REGISTER:
						case FORCE_MULTIPLE_COILS:
						case PRESET_MULTIPLE_REGISTERS:

						this->processSuccess();

						break;
						default: // illegal function returned
						(*DebugPort).println(F("-------- ERROR Illegal Function --------"));
						this->processError();
						break;
					}
				}
			}
			else // checksum failed
			{
				(*DebugPort).println(F("-------- ERROR Checksum Failed --------"));
				this->processError();
			}
		}

		void ModbusIPSerialLoRaLink::processError()
		{
			packet->retries++;
			packet->failed_requests++;

			state = IDLE;
			delayStart = millis(); // start the turnaround delay
		}

		void ModbusIPSerialLoRaLink::processSuccess()
		{
			buffer_len = buffer_len - 3;
			int i = 0;
			if (_reply != MB_REPLY_OFF) {
				//MBAP
				_MBAP[4] = (buffer_len+1) >> 8;     //_len+1 for last byte from MBAP
				_MBAP[5] = (buffer_len+1) & 0x00FF;

				byte sendbuffer[7 + buffer_len];

				for (int i = 0 ; i < 7 ; i++) {
					sendbuffer[i] = _MBAP[i];
				}
				//PDU Frame
				for (int i = 0 ; i < buffer_len+1 ; i++) {
					sendbuffer[i+6] = frame[i];
				}
				// for (int i = 0 ; i < buffer_len+7 ; i++) {
				// 	frame[i] = sendbuffer[i];
				// }

				_client.write(sendbuffer, buffer_len + 7);

				(*DebugPort).println(F("SENT IP RESPONSE"));
				for (i=0 ; i < buffer_len+7 ; i++) {
					(*DebugPort).print(sendbuffer[i]);
					(*DebugPort).print(":");
					//(*DebugPort).println(sendbuffer[i], HEX);
				}
				(*DebugPort).println();
				(*DebugPort).println(F("-----------------"));

				switch (sendbuffer[7]) // check function returned
				{
					case READ_COIL_STATUS:
					(*DebugPort).println(F("-------- FUNTION READ_COIL_STATUS --------"));
					break;
					case READ_INPUT_STATUS:
					(*DebugPort).println(F("-------- FUNTION READ_INPUT_STATUS --------"));
					break;
					case READ_INPUT_REGISTERS:
					(*DebugPort).println(F("-------- FUNTION READ_INPUT_REGISTERS --------"));
					break;
					case READ_HOLDING_REGISTERS:
					(*DebugPort).println(F("-------- FUNTION READ_HOLDING_REGISTERS --------"));
					break;
					case FORCE_SINGLE_COIL:
					(*DebugPort).println(F("-------- FUNTION FORCE_SINGLE_COIL --------"));
					break;
					case PRESET_SINGLE_REGISTER:
					(*DebugPort).println(F("-------- FUNTION PRESET_SINGLE_REGISTER --------"));
					break;
					case FORCE_MULTIPLE_COILS:
					(*DebugPort).println(F("-------- FUNTION FORCE_MULTIPLE_COILS --------"));
					break;
					case PRESET_MULTIPLE_REGISTERS:
					(*DebugPort).println(F("-------- FUNTION PRESET_MULTIPLE_REGISTERS --------"));
					break;

					default: // illegal function returned
					(*DebugPort).println(F("-------- ERROR Illegal Function --------"));
					break;
				}
				(*DebugPort).println(F("-----------------"));
				(*DebugPort).println(F("-------- SUCCESS --------"));
			}

			#ifndef TCP_KEEP_ALIVE
			_client.stop();
			#endif

			state = IDLE;
			packet->successful_requests++; // transaction sent successfully
			packet->retries = 0; // if a request was successful reset the retry counter
			delayStart = millis(); // start the turnaround delay
			// 	}
			// }
		}

		unsigned int ModbusIPSerialLoRaLink::calculateCRC(unsigned char * frameIn, unsigned char bufferSize)
		{
			unsigned char * framePtr = frameIn;

			unsigned int temp, temp2, flag;
			temp = 0xFFFF;
			for (unsigned char i = 0; i < bufferSize; i++)
			{
				temp = temp ^ framePtr[i];
				for (unsigned char j = 1; j <= 8; j++)
				{
					flag = temp & 0x0001;
					temp >>= 1;
					if (flag)
					temp ^= 0xA001;
				}
			}
			// Reverse byte order.
			temp2 = temp >> 8;
			temp = (temp << 8) | temp2;
			temp &= 0xFFFF;
			// the returned value is already swapped
			// crcLo byte is first & crcHi byte is last
			return temp;
		}

		int ModbusIPSerialLoRaLink::send_cmd_lora(char * cmd, bool printCmd, bool printRes, bool doubleRes) {
			unsigned long currentMillis = millis();
			if (printCmd) {
				(*DebugPort).println(F("SENT LoRa Message"));
				(*DebugPort).println(cmd);
			}
			(*LoraPort).print(cmd);
			(*LoraPort).write("\r\n");

			(*LoraPort).flush();
			delayMicroseconds(frameDelay);

			while (!(*LoraPort).available()  && millis() - currentMillis < timeout_lora);

			String str = (*LoraPort).readStringUntil('\n');
			if (printRes) {
				(*DebugPort).print(F("Res: "));
				(*DebugPort).println(str);
			}

			if(doubleRes)
			{
				currentMillis = millis();

				while (!(*LoraPort).available()  && millis() - currentMillis < timeout_lora);

				String str = (*LoraPort).readStringUntil('\n');
				if (printRes) {
					(*DebugPort).print(F("Res: "));
					(*DebugPort).println(str);
				}
			}
			if (printCmd || printRes)
			(*DebugPort).println(F("-----------------"));
			return 0;
		}

		char ModbusIPSerialLoRaLink::hexToC(char c)
		{
			if (c >= '0' && c <= '9')
			return c - '0';
			else if (c >= 'a' && c <= 'f')
			return 10 + c - 'a';
			else if (c >= 'A' && c <= 'F')
			return 10 + c - 'A';
			else return 0;
		}
		char ModbusIPSerialLoRaLink::hexToChar(char c1, char c2) {
			return (hexToC(c1) * 16 + hexToC(c2));
		}
