/* this file contains several functions used in the main code
which were moved to a separate file solely to reduce clutter
in this code (which is the only code which should need to be
modified.
*/

//////////////// include libraries
#include <XBee.h>

//////////////// include header
#include "ccsds_xbee.h"
#include "CCSDS.h"

//////////////// initalize objects
XBee xbee = XBee();

//////////////// initalize variables

// max length of packet's payload (ie data)
#define PKT_MAX_LEN 100

uint8_t _packet_data[PKT_MAX_LEN];
int _bytesread;

uint32_t _SendCtr = 0;
uint32_t _RcvdCtr = 0;
uint32_t _CmdRejCtr = 0;

void printHex(int num, int precision) {
     char tmp[16];
     char format[20];

     sprintf(format, "0x%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
}

int sendAtCommand(AtCommandRequest atRequest) {
/*
Function to send an AT command to the xbee. Used to send
commands to the xbee to initalize it properly. Returns 0 
if command sent sucessfully, 1 if there was a problem.

see reference for available commands:
http://examples.digi.com/wp-content/uploads/2012/07/XBee_ZB_ZigBee_AT_Commands.pdf
*/  

   AtCommandResponse atResponse = AtCommandResponse();
  
  // initalize status variable to failure (will be changed if it passes)
  uint8_t STATUS = 1;
  
  Serial.println(F("Sending cmd to XBee:"));

  // send the command
  xbee.send(atRequest);
  
  // clear the line since sending the packet will print gibberish
  //Serial.println();
  
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(2000)) {
    // got a response!

    // should be an AT command response
    if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      xbee.getResponse().getAtCommandResponse(atResponse);

      if (atResponse.isOk()) {
        Serial.print("Cmd [");
        Serial.print(char(atResponse.getCommand()[0]));
        Serial.print(char(atResponse.getCommand()[1]));
        Serial.print("] sent");

        if (atResponse.getValueLength() > 0) {
          //Serial.print("Command value length is ");
          //Serial.println(atResponse.getValueLength(), DEC);

          Serial.print(" and returned: ");
          for (int i = 0; i < atResponse.getValueLength(); i++) {
            printHex(atResponse.getValue()[i], 2);
            Serial.print(" ");
          }

        }
        
        Serial.println();
        STATUS = 0;
      } 
      else {
        Serial.print(F("Command return error code: "));
        printHex(atResponse.getStatus(), 2);
        Serial.println();
        STATUS = 1;
      }
    } else {
      Serial.print(F("Expected AT response but got "));
      printHex(xbee.getResponse().getApiId(), 2);
      Serial.println();
      STATUS = 1;
    }   
  } else {
    // at command failed
    if (xbee.getResponse().isError()) {
      Serial.print(F("Error reading packet.  Error code: "));  
      printHex(xbee.getResponse().getErrorCode(),2);
      Serial.println();
      STATUS = 1;
    } 
    else {
      Serial.println(F("No response from radio"));  
      STATUS = 1;
    }
  }
  return STATUS;
}


int InitXBee(uint16_t address, uint16_t PanID, Stream &serial) {
/*
Function to initalize the xbee. Does the following things to the attached Xbee:
    Sets MY (16bit address) to the value given in the second argument of InitXBee
    Sets ID (PAN ID) to 0x0B0B (arbitrarily chosen, but all Xbees much match).
    Sets CH (Channel) to C, arbitraryily chosen, but all Xbees must match
    Sets CE (Coordinator Enable), should be 0 for all xbees
    Sets PL (Power level), should be set at max
    Sets BD (Interface Data Rate) to 9600, must match arduino serial baud rate
    Sets AP (API enable) to enabled
  
These values are NOT saved to non-volatile memory, so the changes will be lost 
as soon as the xbee loses power!
*/

  // associate the xbee with the serial
  xbee.setSerial(serial);

  AtCommandRequest atRequest = AtCommandRequest();
    
  /*
  initialize status variable which will be returned indicating sucess
  
  Each step in the process will set one bit in the status value. That bit
  will be set to 0 if that step completed properly, or 1 if it didn't. Thus,
  a final value of 0 indicates complete success whereas a non-zero value 
  indicates which step in the process failed.
  */
  uint16_t STATUS = 0;
  
  // initalize a MY command with desired address
  uint8_t MYCmd[] = {'M','Y'};
  uint8_t MYSetVal[] = { (uint8_t)(address >> 8), (uint8_t)(address & 0xff)};
    
  // initalize an ID command which will set the PAN ID
  uint8_t IDCmd[] = {'I','D'};
  uint8_t IDSetVal[] = { (uint8_t)(PanID >> 8), (uint8_t)(PanID & 0xff) };
  
  // initalize a AC command which will make any changes to settings take effect
  uint8_t ACCmd[] = {'A','C'};
  
  // initalize a WR command which would write the current settings to non-volatile memory
  uint8_t WRCmd[] = {'W','R'};
  
  // initalize a AP command which sets the Xbee to API mode
  uint8_t APCmd[] = {'A','P'};
  uint8_t APSetVal[] = {0x01};
  
  // initalize a CE command which sets the Xbee to end node
  uint8_t CECmd[] = {'C','E'};
  uint8_t CESetVal[] = {0x00};

  // initalize a CH command which sets the Xbee channel to C
  uint8_t CHCmd[] = {'C','H'};
  uint8_t CHSetVal[] = {0x0C};

  // initalize a BD command which sets the Xbee baud rate to 9600
  uint8_t BDCmd[] = {'B','D'};
  uint8_t BDSetVal[] = {0x03};

  // initalize a PL command which sets the Xbee power level to max
  uint8_t PLCmd[] = {'P','L'};
  uint8_t PLSetVal[] = {0x04};

  
  // send the MY command to set the address
  atRequest = AtCommandRequest(MYCmd, MYSetVal, sizeof(MYSetVal));
  STATUS |= sendAtCommand(atRequest);
  
  // sent the AC command to apply changes
  atRequest = AtCommandRequest(ACCmd);  
  STATUS |= sendAtCommand(atRequest) << 1;
  
  // send a command to read the current MY address
  atRequest = AtCommandRequest(MYCmd);  
  STATUS |= sendAtCommand(atRequest) << 2;
  
  // sent a command to set the PAN ID
  atRequest = AtCommandRequest(IDCmd, IDSetVal, sizeof(IDSetVal));  
  STATUS |= sendAtCommand(atRequest) << 3;
  
  // send a command to apply changes
  atRequest = AtCommandRequest(ACCmd);  
  STATUS |= sendAtCommand(atRequest) << 4;
  
  // send a command to read the PAN ID
  atRequest = AtCommandRequest(IDCmd);  
  STATUS |= sendAtCommand(atRequest) << 5;
  
  // send a command to set the API mode
  atRequest = AtCommandRequest(APCmd, APSetVal, sizeof(APSetVal));   
  STATUS |= sendAtCommand(atRequest) << 6;
  
  // send a command to read the PAN ID
  atRequest = AtCommandRequest(CECmd, CESetVal, sizeof(CESetVal));   
  STATUS |= sendAtCommand(atRequest) << 7;
  
  // send a command to read the PAN ID
  atRequest = AtCommandRequest(CHCmd, CHSetVal, sizeof(CHSetVal));   
  STATUS |= sendAtCommand(atRequest) << 8;
  
  // send a command to read the PAN ID
  atRequest = AtCommandRequest(BDCmd, BDSetVal, sizeof(BDSetVal));   
  STATUS |= sendAtCommand(atRequest) << 9;
  
  // send a command to read the PAN ID
  atRequest = AtCommandRequest(PLCmd, PLSetVal, sizeof(PLSetVal));   
  STATUS |= sendAtCommand(atRequest) << 10;
  
  // return the status variable
  return STATUS;
}

int _sendData(uint16_t SendAddr, uint8_t payload[], int payload_size){
/* 
Function to send data to a remote xbee. Takes as arguments the address
of the remote xbee to send to, the data to be sent, and the size of the 
data to be sent.

Will send the data and print the SendCtr and the data sent to the serial.
*/ 
  
  
  // create the message to be sent
  Tx16Request tx = Tx16Request(SendAddr, payload, payload_size);
  
  // Send the data.
  
  // Note that the Xbee will automatically try to resend the packet 3 times if it 
  // does not receive and ACK from the Logger.
  xbee.send(tx);
  
  // keep track of how many packets we've sent
  _SendCtr++;
  
  // Display data for the user
  Serial.println();
  Serial.print(F("Sent pkt #"));
  Serial.print(_SendCtr);
  Serial.print(F(", data: "));

  for(int i = 0; i < payload_size; i++){
    printHex(payload[i],2);
    Serial.print(", ");
  }
  Serial.println();

}

int sendTlmMsg(uint16_t SendAddr, uint8_t payload[], int payload_size){

  // if the user attempts to send a packet that's too long, don't do it
  if(payload_size+12 > PKT_MAX_LEN){
    Serial.println("Packet too long... not sending");
    return -1;
  }

  // allocate the buffer to compile the packet in
  uint8_t _packet_data[PKT_MAX_LEN];
  uint8_t _payload_size = payload_size;
  
  // declare the header structures
  CCSDS_PriHdr_t _PriHeader;
  CCSDS_TlmSecHdr_t _TlmSecHeader;

  // fill primary header fields
  CCSDS_WR_APID(_PriHeader,SendAddr);
  CCSDS_WR_SHDR(_PriHeader,1);
  CCSDS_WR_TYPE(_PriHeader,0);
  CCSDS_WR_VERS(_PriHeader,0);
  CCSDS_WR_SEQ(_PriHeader,_SendCtr);
  CCSDS_WR_SEQFLG(_PriHeader,0x03);
  CCSDS_WR_LEN(_PriHeader,payload_size+sizeof(_PriHeader)+sizeof(_TlmSecHeader));

  // fill secondary header fields
  CCSDS_WR_SEC_HDR_SEC(_TlmSecHeader,millis()/1000L);
  CCSDS_WR_SEC_HDR_SUBSEC(_TlmSecHeader,millis() % 1000L);
  
  // copy the primary header
  memcpy(_packet_data, &_PriHeader, sizeof(_PriHeader));
  _payload_size += sizeof(_PriHeader);

  // copy the secondary header
  memcpy(_packet_data+sizeof(_PriHeader), &_TlmSecHeader, sizeof(_TlmSecHeader));
  _payload_size += sizeof(_TlmSecHeader);

  // copy the packet data
  memcpy(_packet_data+sizeof(_PriHeader)+sizeof(_TlmSecHeader), payload, _payload_size);

  // send the message
  _sendData(SendAddr, _packet_data, _payload_size);

  return 1;
}


int sendCmdMsg(uint16_t SendAddr, uint8_t fcncode, uint8_t payload[], int payload_size){

  uint8_t _payload_size = payload_size;

  // if the user attempts to send a packet that's too long, don't do it
  if(payload_size +12 > PKT_MAX_LEN){
    Serial.println("Packet too long... not sending");
    return -1;
  }

  // allocate the buffer to compile the packet in
  uint8_t packet_data[PKT_MAX_LEN];

  // declare the header structures
  CCSDS_PriHdr_t _PriHeader;
  CCSDS_CmdSecHdr_t _CmdSecHeader;

  // fill primary header fields
  CCSDS_WR_APID(_PriHeader,SendAddr);
  CCSDS_WR_SHDR(_PriHeader,1);
  CCSDS_WR_TYPE(_PriHeader,1);
  CCSDS_WR_VERS(_PriHeader,0);
  CCSDS_WR_SEQ(_PriHeader,_SendCtr);
  CCSDS_WR_SEQFLG(_PriHeader,0x03);
  CCSDS_WR_LEN(_PriHeader,payload_size+sizeof(_PriHeader)+sizeof(_CmdSecHeader));

  // fill secondary header fields
  CCSDS_WR_CHECKSUM(_CmdSecHeader, 0x0);
  CCSDS_WR_FC(_CmdSecHeader, fcncode);
  
  // copy the primary header
  memcpy(packet_data, &_PriHeader, sizeof(_PriHeader));
  _payload_size += sizeof(_PriHeader);

  // copy the secondary header
  memcpy(packet_data+sizeof(_PriHeader), &_CmdSecHeader, sizeof(_CmdSecHeader));
  _payload_size += sizeof(_CmdSecHeader);

  // write the checksum after the header's been added
  CCSDS_WR_CHECKSUM((*(CCSDS_CmdPkt_t*) _packet_data).SecHdr, CCSDS_ComputeCheckSum((CCSDS_CmdPkt_t*) _packet_data));

  // copy the packet data
  memcpy(packet_data+sizeof(_PriHeader)+sizeof(_CmdSecHeader), payload, _payload_size);

  // send the message
  _sendData(SendAddr, packet_data, _payload_size);

  return 1;
}

int readMsg(uint16_t timeout){
  
  // read a message from the xbee
  _bytesread = _readXbeeMsg(_packet_data, timeout);
        
  if(_bytesread > 0){
    
    // declare the header structures
    CCSDS_PriHdr_t _PriHeader;
    
    // cast the primary header into the header structure
     _PriHeader = *(CCSDS_PriHdr_t*) (_packet_data);

    // return the packet type
    return CCSDS_RD_TYPE(_PriHeader);
  }
  else{
    // if an error occured, print it
    if(_bytesread < -1){
      Serial.print("Error reading, code: ");
      Serial.print(_bytesread);
    }
    return -1;
  }
}

int readCmdMsg(uint8_t params[], uint8_t &fcncode){

  /*
  if(CCSDS_ValidCheckSum ((CCSDS_CmdPkt_t*) (_packet_data))){
    Serial.println("Invalid");
    _CmdRejCtr++;
    return -1;
  }
*/

  // declare the header structures
  CCSDS_PriHdr_t _PriHeader;

  // copy the primary header into a structure
  //memcpy(&_PriHeader, _packet_data, sizeof(_PriHeader));
  _PriHeader = *(CCSDS_PriHdr_t*) (_packet_data);

  if(CCSDS_RD_SHDR(_PriHeader)){

    // declare the header structures
    CCSDS_CmdSecHdr_t _CmdSecHeader;

    // copy the secondary header into a structure
    //memcpy(&_CmdSecHeader, _packet_data+sizeof(_PriHeader), sizeof(_CmdSecHeader));
    _CmdSecHeader = *(CCSDS_CmdSecHdr_t*) (_packet_data+sizeof(_PriHeader));

    // copy the parameters into the user's pointer
    memcpy(params, _packet_data+sizeof(_PriHeader)+sizeof(_CmdSecHeader), _bytesread-sizeof(_PriHeader)-sizeof(_CmdSecHeader));

    fcncode = CCSDS_RD_FC(_CmdSecHeader);

    // return param length
    return CCSDS_RD_LEN(_PriHeader)-sizeof(_PriHeader)-sizeof(_CmdSecHeader);
        
  }
}

int readTlmMsg(uint8_t data[]){
  
  // declare the header structures
  CCSDS_PriHdr_t _PriHeader;

  // copy the primary header into a structure
  //memcpy(&_PriHeader, _packet_data, sizeof(_PriHeader));
  _PriHeader = *(CCSDS_PriHdr_t*) (_packet_data);

  if(!CCSDS_RD_SHDR(_PriHeader)){

    // declare the header structures
    CCSDS_TlmSecHdr_t _TlmSecHeader;

    // copy the secondary header into a structure
    //memcpy(&_TlmSecHeader, _packet_data+sizeof(_PriHeader), sizeof(_TlmSecHeader));
    _TlmSecHeader = *(CCSDS_TlmSecHdr_t*) (_packet_data+sizeof(_PriHeader));

    // copy the parameters into the user's pointer
    memcpy(data, _packet_data+sizeof(_PriHeader)+sizeof(_TlmSecHeader), _bytesread-sizeof(_PriHeader)-sizeof(_TlmSecHeader));

    // return param length
    return CCSDS_RD_LEN(_PriHeader)-sizeof(_PriHeader)-sizeof(_TlmSecHeader);

   }
}

void printPktInfo(){

  // declare the primary header structure
  CCSDS_PriHdr_t _PriHeader;

  // copy data to it
  //memcpy(&_PriHeader, _packet_data, sizeof(_PriHeader));
  _PriHeader = *(CCSDS_PriHdr_t*) (_packet_data);

  // print info about the packet
  Serial.print("APID: ");
  Serial.print(CCSDS_RD_APID(_PriHeader));
  Serial.print(", SecHdr: ");
  Serial.print(CCSDS_RD_SHDR(_PriHeader));
  Serial.print(", Type: ");
  Serial.print(CCSDS_RD_TYPE(_PriHeader));
  Serial.print(", Ver: ");
  Serial.print(CCSDS_RD_VERS(_PriHeader));
  Serial.print(", SeqCnt: ");
  Serial.print(CCSDS_RD_SEQ(_PriHeader));
  Serial.print(", SegFlag: ");
  Serial.print(CCSDS_RD_SEQFLG(_PriHeader));
  Serial.print(", Len: ");
  Serial.println(CCSDS_RD_LEN(_PriHeader));

  // process command and telemetry secondary headers
  if(CCSDS_RD_TYPE(_PriHeader)){

    // declare the command secondary header structure
    CCSDS_CmdSecHdr_t _CmdSecHeader;

    // copy the data to it
    //memcpy(&_CmdSecHeader, _packet_data+sizeof(_PriHeader), sizeof(_CmdSecHeader));
    _CmdSecHeader = *(CCSDS_CmdSecHdr_t*) (_packet_data+sizeof(_PriHeader));

    // print the command-specific data
    Serial.print("FcnCode: ");
    Serial.print(CCSDS_RD_FC(_CmdSecHeader));
    Serial.print(", CkSum: ");
    Serial.println(CCSDS_RD_CHECKSUM(_CmdSecHeader));
  }
  else{

    // declare the telemetry secondary header structure
    CCSDS_TlmSecHdr_t _TlmSecHeader;

    // copy the data to it
    //memcpy(&_TlmSecHeader, _packet_data+sizeof(_PriHeader), sizeof(_TlmSecHeader));
    _TlmSecHeader = *(CCSDS_TlmSecHdr_t*) (_packet_data+sizeof(_PriHeader));

    // print the telemetry-specific data
    Serial.print("Sec: ");
    Serial.print(CCSDS_RD_SEC_HDR_SEC(_TlmSecHeader));
    Serial.print("Subsec: ");
    Serial.println(CCSDS_RD_SEC_HDR_SUBSEC(_TlmSecHeader));
  }   
}

int _readXbeeMsg(uint8_t data[], uint16_t timeout){
/*
Reads a message from the xbee, checks if its a ACK, and if not, extracts the data and 
returns it.

Returns either:
   The length of the data read (note, the total length of the array will be 4 elements longer 
     than that.
   -1 if not packet was not available.
   0 if an non-data packet was read (ie, an ACK packet, or another type).
   A negative number (other than -1) if reading the packet produced an error. The number 
     is the error code.

Note that this function uses a blocking form of readPacket and will return when either a
new packet has been read or the timeout has expired. Be careful with the time you use and
its effect on the rest of the program.

*/

  // readpacket doesn't work when the timeout is 0, prevent the user from doing something stupid
  if(timeout == 0){
    timeout = 1;
  }
    
  // wait for the message
  // NOTE: THIS IS BLOCKING
  if (xbee.readPacket(timeout)){
     //Serial.println("Read pkt ");
      // if its a znet tx status            	
  	if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
          //Serial.print(F("Received response..."));
                
      // create response object for the message we're sending
      TxStatusResponse txStatus = TxStatusResponse();

  	   xbee.getResponse().getZBTxStatusResponse(txStatus);
  		
  	   // get the delivery status, the fifth byte
         if (txStatus.getStatus() == SUCCESS) {
          	// success.  time to celebrate
                //Serial.print(F("ACK!"));
                return 0;
         } else {
          	// the remote XBee did not receive our packet. sadface.
                //Serial.print(F("Not ACK"));
                return 0;
         }
      }
     else if( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
       
      // record the new packet
      _RcvdCtr++;
      
      Serial.print(F("Received Message..."));
       
      // object for holding packets received over xbee
      Rx16Response reponse16 = Rx16Response();
      
      // read the packet
      xbee.getResponse().getRx16Response(reponse16);
        
      // copy data from packet into the data array starting at element 4
      memcpy(data, reponse16.getData(), reponse16.getDataLength());
      
      return reponse16.getDataLength();

     }
     else{
       Serial.print(F("Received something else"));
       return 0;
     }   
     Serial.println();
  } else if (xbee.getResponse().isError()) {
    
    // tell the user that there was an error
    Serial.print(F("Read Error, Error Code:"));
    Serial.println(xbee.getResponse().getErrorCode());
    
    // return the error code
    return -xbee.getResponse().getErrorCode();
    
  } else {
    /*
    This typically means there was no packet to read, thogh can also mean
    that the xbee is hooked up correctly (though that should've been detected
    in the initalization
    */
    //Serial.println("No packet available");    
    return -1;
  } 
}

