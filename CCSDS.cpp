#include "CCSDS.h"

bool CCSDS_ValidCheckSum (CCSDS_CmdPkt_t *PktPtr)
{
   return (CCSDS_ComputeCheckSum(PktPtr) == 0);
  // This works because when you XOR something with itself, it will be zero. 
  // Checksum part becomes zero, and XOR with zero is identity, so it is as if 
  // that byte was not there for the initial checksum.

} /* END CCSDS_ValidCheckSum() */


uint8_t CCSDS_ComputeCheckSum (CCSDS_CmdPkt_t *PktPtr)
{
   uint16_t   PktLen   = CCSDS_RD_LEN(PktPtr->PriHdr);
   uint8_t   *BytePtr  = (uint8_t *)PktPtr;
   uint8_t    CheckSum;

   CheckSum = 0xFF;
   while (PktLen--)  CheckSum ^= *(BytePtr++);

   return CheckSum;

} /* END CCSDS_ComputeCheckSum() */
