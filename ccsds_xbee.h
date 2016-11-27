#ifndef _ccsds_xbee_
#define _ccsds_xbee_


//////////////////////////////////////////////////////////////
//                     Library Includes                     //
//////////////////////////////////////////////////////////////
#include <XBee.h>
#include "CCSDS.h"

//////////////////////////////////////////////////////////////
//                     Opt-Out Library Includes             //
//////////////////////////////////////////////////////////////
/*
 * The user may define the macros '_NO_RTC_' or '_NO_SD_' to disable
 * the capability of using the RTC or logging and not have those
 * libraries contribute to the size of the sketch
 */
#ifndef _NO_RTC_
#include "RTClib.h"  // RTC and SoftRTC
#endif

#ifndef _NO_SD_
#include <SD.h>
#endif


//////////////////////////////////////////////////////////////
//                     Compile-Time Configuration           //
//////////////////////////////////////////////////////////////
// define the maximum expected length of packet's payload (ie data) to initalize buffer 
// FIXME: cite a source on the origin of this number. look in xbee documentation
#ifndef PKT_MAX_LEN
#define PKT_MAX_LEN 100
#endif


//////////////////////////////////////////////////////////////
//                     Enumerations                         //
//////////////////////////////////////////////////////////////

// Pkt Types
#define CCSDS_CMD_PKT 1
#define CCSDS_TLM_PKT 0

#define CCSDS_SUCCESS 1
#define CCSDS_NO_RCVD 0
#define CCSDS_FAILURE -1

#define LOG_RCVD 1
#define LOG_SENT 0
#define LOG_NONE -1


//////////////////////////////////////////////////////////////
//                     Public Methods                       //
//////////////////////////////////////////////////////////////
class CCSDS_Xbee
{
  
//////////////////////////////////////////////////////////////
//                     Public Methods                       //
//////////////////////////////////////////////////////////////
  public:
    CCSDS_Xbee();
	
    // Initalization functions
    int init(uint16_t address, uint16_t PanID, Stream &xbee_serial);
    int init(uint16_t address, uint16_t PanID, Stream &xbee_serial, Stream &debug_serial);

#ifndef _NO_SD_
    int start_logging(File logfile);
    int end_logging();
#endif
#ifndef _NO_RTC_
    int add_rtc(RTC_DS1307 rtc);
    int remove_rtc();
#endif
    int add_debug_serial(Stream &debug_serial);
    int remove_debug_serial();

    // sending functions
    int sendAtCommand(AtCommandRequest atRequest);
    int createTlmMsg(uint8_t pkt_buf[], uint16_t _APID, uint8_t _payload[], uint16_t _payload_size);
    int sendTlmMsg(uint16_t SendAddr, uint16_t APID, uint8_t payload[], uint16_t payload_size);
    int createCmdMsg(uint8_t pkt_buf[], uint16_t APID, uint8_t FcnCode, uint8_t payload[], uint16_t payload_size);
    int sendCmdMsg(uint16_t SendAddr, uint16_t APID, uint8_t fcncode, uint8_t payload[], uint16_t payload_size);

    // reading functions
    int readMsg(uint8_t packet_data[], uint16_t timeout);
    int readMsg(uint8_t packet_data[]);

    // utility functions
    void printHex(int num, uint8_t precision);
    void printPktInfo(CCSDS_PriHdr_t &_PriHeader);
#ifndef _NO_SD_
    void logPkt(File logfile, uint8_t data[], uint8_t len, uint8_t received_flg);
#endif

    // counter getters 
    uint32_t getSentByteCtr();
    uint32_t getSentPktCtr();
    uint32_t getRcvdByteCtr();
    uint32_t getRcvdPktCtr();
    uint8_t getPrevRSSI();
    uint8_t getPrevSenderAddr();
    
    // counter resetters
    void resetSentByteCtr();
    void resetRcvdByteCtr();
    void resetSentPktCtr();
    void resetRcvdPktCtr();
    void resetCounters();
  
    // reading functions
    int _readXbeeMsg(uint8_t data[]);
    int _readXbeeMsg(uint8_t data[], uint16_t timeout);
    // sending functions
    void sendRawData(uint16_t SendAddr, uint8_t payload[], uint16_t payload_size);
	
//////////////////////////////////////////////////////////////
//                     Private Methods                      //
//////////////////////////////////////////////////////////////
  private:
    // initalize an Xbee object from the adafruit xbee library
    XBee xbee = XBee();
	
#ifndef _NO_RTC_
    // RTC used for time tagging packets
    bool _rtc_defined = false;
    RTC_DS1307 _rtc;
#endif

    // flag controlling if debugging statements are printed to Serial0
    // set to false by default, set by user by calling 4 argument InitXbee
    bool _debug_serial_defined = false;
    Stream* _debug_serial;

#ifndef _NO_SD_
    // Log file 
    bool _logfile_defined = false;
    File _logfile;
#endif

    // initalize counter to hold number of bytes read
    uint32_t _bytesread;
    uint32_t _bytessent;

    // initalize counters to track packet I/O
    uint32_t _SentPktCtr = 0;
    uint32_t _RcvdPktCtr = 0;
    uint32_t _SentByteCtr = 0;
    uint32_t _RcvdByteCtr = 0;
    
    uint8_t _PrevPktRSSI = 0;
    uint8_t _PrevSenderAddr = 0;
    
    void print_time(File logfile);
#ifndef _NO_SD_
    void logPkt(uint8_t data[], uint8_t len, uint8_t received_flg);
#endif
};

//////////////////////////////////////////////////////////////
//                     Utility Functions                    //
//////////////////////////////////////////////////////////////
// utility functions for getting/setting fields of CCSDS packets
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

uint8_t validateChecksum(uint8_t _packet[]);
uint16_t computeChecksum(uint8_t _packet[]);

CCSDS_PriHdr_t getPrimaryHeader(uint8_t _packet[]);
CCSDS_TlmSecHdr_t getTlmHeader(uint8_t _packet[]);
CCSDS_CmdSecHdr_t getCmdHeader(uint8_t _packet[]);
CCSDS_TlmPkt_t getTlmPkt(uint8_t _packet[]);
CCSDS_CmdPkt_t getCmdPkt(uint8_t _packet[]);


//////////////////////////////////////////////////////////////
//                     Utility Functions                    //
//////////////////////////////////////////////////////////////
// utility functions for adding telemetry to buffers

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

uint8_t addStrToTlm(char *s, uint8_t payload[], uint8_t start_pos);
//uint8_t addStrToTlm(const String &s, uint8_t payload[], uint8_t start_pos);

#endif  /* _ccsds_xbee_class_ */