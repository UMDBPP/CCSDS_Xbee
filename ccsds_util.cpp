#include "ccsds_util.h"

uint16_t getAPID(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_APID(header);
}

// 0 for TLM packet, 1 for CMD packet
uint8_t getPacketType(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_TYPE(header);
}

int getPacketLength(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_LEN(header);
}

CCSDS_PriHdr_t getPrimaryHeader(uint8_t _packet[]) {
	return *(CCSDS_PriHdr_t*)(_packet);
}

CCSDS_TlmSecHdr_t getTlmHeader(uint8_t _packet[]) {
	return *(CCSDS_TlmSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));
}

CCSDS_CmdSecHdr_t getCmdHeader(uint8_t _packet[]) {
	return *(CCSDS_CmdSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));
}
