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
uint8_t getPacketType(uint8_t _packet[]);
int getPacketLength(uint8_t _packet[]);

CCSDS_PriHdr_t getPrimaryHeader(uint8_t _packet[]);
CCSDS_TlmSecHdr_t getTlmHeader(uint8_t _packet[]);
CCSDS_CmdSecHdr_t getCmdHeader(uint8_t _packet[]);

#endif // CCSDS_UTIL_H
