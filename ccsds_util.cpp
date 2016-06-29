#include "ccsds_util.h"

uint16_t getAPID(uint8_t _packet[], int _packetLength) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet, _packetLength);

	return CCSDS_RD_APID(header);
}

CCSDS_PriHdr_t getPrimaryHeader(uint8_t _packet[], int _packetLength) {
	return *(CCSDS_PriHdr_t*)(_packet);
}

// 0 for TLM packet, 1 for CMD packet
uint8_t getPacketType(uint8_t _packet[], int _packetLength) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet, _packetLength);

	return CCSDS_RD_TYPE(header);
}
