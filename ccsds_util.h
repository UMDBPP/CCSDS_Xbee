/*
Contains several useful functions for interacting with CCSDS.
These are mostly to smooth over 900Relay (Data Parrot)'s transition to the
updated XBee library.
Gets several useful bits and bobs directly from the raw packet data.
*/

#ifndef CCSDS_UTIL_H
#define CCSDS_UTIL_H

#include "CCSDS.h"

uint16_t getAPID(uint8_t _packet[]);
void setAPID(uint8_t _packet[], uint16_t APID);

uint8_t getSecHdrFlg(uint8_t _packet[]);
void setSecHdrFlg(uint8_t _packet[], uint8_t SHDR);

uint8_t getVer(uint8_t _packet[]);
void setVer(uint8_t _packet[], uint8_t ver);

uint16_t getSeqCtr(uint8_t _packet[]);
void setSeqCtr(uint8_t _packet[], uint16_t seqctr);

uint8_t getSeqFlg(uint8_t _packet[]);
void setSeqFlg(uint8_t _packet[], uint8_t seqflg);

uint8_t getPacketType(uint8_t _packet[]);
void setPacketType(uint8_t _packet[], uint8_t Type);

uint16_t getPacketLength(uint8_t _packet[]);
void setPacketLength(uint8_t _packet[], uint16_t Len);

uint32_t getTlmTimeSec(uint8_t _packet[]);
void setTlmTimeSec(uint8_t _packet[], uint32_t sec);

uint16_t getTlmTimeSubSec(uint8_t _packet[]);
void setTlmTimeSubSec(uint8_t _packet[], uint16_t subsec);

uint8_t getCmdFunctionCode(uint8_t _packet[]);
void setCmdFunctionCode(uint8_t _packet[], uint8_t fcncode);

uint8_t getCmdChecksum(uint8_t _packet[]);
void setCmdChecksum(uint8_t _packet[], uint8_t checksum);

uint8_t validateChecksum(uint8_t _packet[])

CCSDS_PriHdr_t getPrimaryHeader(uint8_t _packet[]);
CCSDS_TlmSecHdr_t getTlmHeader(uint8_t _packet[]);
CCSDS_CmdSecHdr_t getCmdHeader(uint8_t _packet[]);

#endif // CCSDS_UTIL_H
