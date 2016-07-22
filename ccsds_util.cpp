#include "ccsds_util.h"

uint16_t getAPID(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_APID(header);
}

void setAPID(uint8_t _packet[], uint16_t APID) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_APID((*header),APID);
}

uint8_t getSecHdrFlg(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_SHDR(header);
}

void setSecHdrFlg(uint8_t _packet[], uint8_t SHDR) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_SHDR((*header),SHDR);
}

uint8_t getVer(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_VERS(header);
}

void setVer(uint8_t _packet[], uint8_t ver) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_VERS((*header),ver);
}

uint16_t getSeqCtr(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_SEQ(header);
}

void setSeqCtr(uint8_t _packet[], uint16_t seqctr) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_SEQ((*header),seqctr);
}

uint8_t getSeqFlg(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_SEQFLG(header);
}

void setSeqFlg(uint8_t _packet[], uint8_t seqflg) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_SEQFLG((*header),seqflg);
}

// 0 for TLM packet, 1 for CMD packet
uint8_t getPacketType(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_TYPE(header);
}

void setPacketType(uint8_t _packet[], uint8_t Type) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_TYPE((*header),Type);
}

uint16_t getPacketLength(uint8_t _packet[]) {
	CCSDS_PriHdr_t header = getPrimaryHeader(_packet);

	return CCSDS_RD_LEN(header);
}

void setPacketLength(uint8_t _packet[], uint16_t Len) {
	CCSDS_PriHdr_t *header = (CCSDS_PriHdr_t*)_packet;

	CCSDS_WR_LEN((*header), Len);
}


uint32_t getTlmTimeSec(uint8_t _packet[]) {
	CCSDS_TlmSecHdr_t header = getTlmHeader(_packet);

	return CCSDS_RD_SEC_HDR_SEC(header);
}

void setTlmTimeSec(uint8_t _packet[], uint32_t sec) {
	CCSDS_TlmSecHdr_t *header = (CCSDS_TlmSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));

	CCSDS_WR_SEC_HDR_SEC((*header), sec);
}

uint16_t getTlmTimeSubSec(uint8_t _packet[]) {
	CCSDS_TlmSecHdr_t header = getTlmHeader(_packet);

	return CCSDS_RD_SEC_HDR_SUBSEC(header);
}

void setTlmTimeSubSec(uint8_t _packet[], uint16_t subsec) {
	CCSDS_TlmSecHdr_t *header = (CCSDS_TlmSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));

	CCSDS_WR_SEC_HDR_SUBSEC((*header), subsec);
}

uint8_t getCmdFunctionCode(uint8_t _packet[]) {
	CCSDS_CmdSecHdr_t shdr = getCmdHeader(_packet);

	return CCSDS_RD_FC(shdr);
}

void setCmdFunctionCode(uint8_t _packet[], uint8_t fcncode) {
	CCSDS_CmdSecHdr_t *shdr = (CCSDS_CmdSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));

	CCSDS_WR_FC((*shdr), fcncode);
}

uint8_t getCmdChecksum(uint8_t _packet[]) {
	CCSDS_CmdSecHdr_t shdr = getCmdHeader(_packet);

	return CCSDS_RD_CHECKSUM(shdr);
}

void setCmdChecksum(uint8_t _packet[], uint8_t checksum) {
	CCSDS_CmdSecHdr_t *shdr = (CCSDS_CmdSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));

	CCSDS_WR_CHECKSUM((*shdr), checksum);
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
