#ifndef _ccsds_xbee_
#define _ccsds_xbee_

//#include <Serial.h>
#include <XBee.h>

// Initalization functions
int InitXBee(uint16_t address,  uint16_t PanID, Stream &serial);

// sending functions
int sendAtCommand(AtCommandRequest atRequest);

int _sendData(uint16_t SendAddr, uint8_t payload[], int payload_size);
int sendTlmMsg(uint16_t SendAddr, uint8_t payload[], int payload_size);
int sendCmdMsg(uint16_t SendAddr, uint8_t fcncode, uint8_t payload[], int payload_size);

// reading functions
int _readXbeeMsg(uint8_t data[], uint16_t timeout);
int readMsg(uint16_t timeout);
int readCmdMsg(uint8_t params[], uint8_t &fcncode);
int readTlmMsg(uint8_t data[]);

// utility functions

void printPktInfo();

template<typename T> uint8_t addIntToTlm(const T& val, uint8_t payload[], uint8_t start_pos) {
	for(uint8_t i = 0; i < sizeof(T); i++) { // Loop through each byte
		uint8_t shiftBits = 8 * (sizeof(T) - (i + 1)); // Number of bits to right shift
		payload[start_pos + i] = (val >> shiftBits) & 0xFF; // Assign value
	}
	return start_pos + sizeof(T); // Return new array index.
}

template<typename T> uint8_t addFloatToTlm(const T& val, uint8_t payload[], uint8_t start_pos) {
	uint32_t tmp = 0x00000000;
	memcpy(&tmp,&val,sizeof(T));
	payload[start_pos] = (tmp >> 24) & 0xFF;
    payload[start_pos+1] = (tmp >> 16) & 0xFF;
    payload[start_pos+2] = (tmp >> 8) & 0xFF;
    payload[start_pos+3] = (tmp >> 0) & 0xFF;
    return start_pos + 4;
}

template<typename T> uint8_t extractFromTlm(T& extractedVal, uint8_t data[], uint8_t start_pos) {
	uint8_t tmp[sizeof(T)];

	for(uint8_t i = 0; i < sizeof(T); i++) {
		tmp[sizeof(T) - (i + 1)] = data[start_pos + i]; // endianness...
    //tmp[i] = data[start_pos + i]; // endianness...
  }
	memcpy(&extractedVal,&tmp[0],sizeof(T));
  return start_pos + sizeof(T);
}


#endif  /* _ccsds_ */

