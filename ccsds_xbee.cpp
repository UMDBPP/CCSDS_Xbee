#include "ccsds_xbee.h"

CCSDS_Xbee::CCSDS_Xbee(){
	
}

int CCSDS_Xbee::init(uint16_t address, uint16_t PanID, Stream &xbee_serial, Stream &debug_serial){
  add_debug_serial(debug_serial);
  return init(address, PanID, xbee_serial);
}

#ifndef _NO_SD_
int CCSDS_Xbee::start_logging(File logfile){
  this->_logfile = logfile;
	this->_logfile_defined = true;
  return CCSDS_SUCCESS;
}

int CCSDS_Xbee::end_logging(){
	this->_logfile_defined = false;
  return CCSDS_SUCCESS;
}
#endif
#ifndef _NO_RTC_
int CCSDS_Xbee::add_rtc(RTC_DS1307 rtc){
  this->_rtc = rtc;
	this->_rtc_defined = true;
  return CCSDS_SUCCESS;
}

int CCSDS_Xbee::remove_rtc(){
	this->_rtc_defined = false;
  return CCSDS_SUCCESS;
}
#endif

int CCSDS_Xbee::add_debug_serial(Stream &debug_serial){
 	this->_debug_serial = &debug_serial;
	this->_debug_serial_defined = true; 
  return CCSDS_SUCCESS;
}

int CCSDS_Xbee::remove_debug_serial(){
	this->_debug_serial_defined = false; 
  return CCSDS_SUCCESS;
}

uint32_t CCSDS_Xbee::getSentByteCtr(){
  return _SentByteCtr;
}

uint32_t CCSDS_Xbee::getRcvdByteCtr(){
  return _RcvdByteCtr;
}

uint32_t CCSDS_Xbee::getRcvdPktCtr(){
  return _RcvdPktCtr;
}

uint8_t CCSDS_Xbee::getPrevRSSI(){
  return _PrevPktRSSI;
}


uint32_t CCSDS_Xbee::getSentPktCtr(){
  return _SentPktCtr;
}

void CCSDS_Xbee::resetRcvdByteCtr(){
  _RcvdByteCtr = 0;
}

void CCSDS_Xbee::resetSentByteCtr(){
  _SentByteCtr = 0;
}

void CCSDS_Xbee::resetSentPktCtr(){
  _SentPktCtr = 0;
}

void CCSDS_Xbee::resetRcvdPktCtr(){
  _SentPktCtr = 0;
}

void CCSDS_Xbee::resetCounters(){
  resetSentByteCtr();
  resetSentByteCtr();
  resetSentPktCtr();
  resetRcvdPktCtr();
}

int CCSDS_Xbee::init(uint16_t address, uint16_t PanID, Stream &xbee_serial) {
/*
Initalizes the the xbee on xbee_serial and sets the following configuration parameters:
    Sets MY (16bit address) to the value given of the first argument (must be unique for each xbee)
    Sets ID (PAN ID) to to the value of the second argument (all Xbees much match)
    Sets CH (Channel) to C, arbitrarily chosen (all Xbees must match)
    Sets CE (Coordinator Enable), should be 0 for all xbees
    Sets PL (Power level), should be set at max
    Sets BD (Interface Data Rate) to 9600, must match arduino serial baud rate
    Sets AP (API enable) to enabled
	
These settings will allow for for packetized communication. The settings are save to non-volatile memory
on the xbee and should persist after its powered off. Initalizing the xbee with this 
function will allow the user to specify if the library should print debug info to Serial0.

Inputs: 
uint16_t address - the 16bit address for the xbee (must be unqiue among xbees)
uint16_t PanID - the 16bit PanID for the xbee (must match other xbees)
Stream &xbee_serial - the serial port the xbee is attached to (Serial, Serial1, Serial2, Serial3)

Outputs:
None

Return:
bitfield indicating the status of the xbee configuration commands

Globals:
None

example:
  InitXBee(0x0002, 0x0B0B, Serial3)

*/	

	// associate the xbee with the serial
	xbee.setSerial(xbee_serial);

	// initalize an ATCommandRequest packet
	AtCommandRequest atRequest = AtCommandRequest();

	/*
	initialize status variable which will be returned indicating sucess

	Each step in the process will set one bit in the status value. That bit
	will be set to 0 if that step completed properly, or 1 if it didn't. Thus,
	a final value of 0 indicates complete success whereas a non-zero value 
	indicates which step in the process failed.
	*/
	uint16_t STATUS = 0;

	// initalize a MY command with user-supplied address
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
	uint8_t APSetVal[] = {0x02};
	// Note: this should be 2 soas to put the xbee in API mode w/PPP. This mode will automatically
	// add escape characters to the message where necessary to make sure the data reaches the other
	// xbee. Mode 1 requires that the user sanitize their message to ensure that none of the xbee
	// control sequences are included in the data to be sent.

	// initalize a CE command which sets the Xbee to end node
	uint8_t CECmd[] = {'C','E'};
	uint8_t CESetVal[] = {0x00};

	// initalize a CH command which sets the Xbee channel to C
	uint8_t CHCmd[] = {'C','H'};
	uint8_t CHSetVal[] = {0x0C};

	// initalize a BD command which sets the Xbee baud rate to 9600
	uint8_t BDCmd[] = {'B','D'};
	uint8_t BDSetVal[] = {0x03};
	// FIXME: allow the user to set baud rate

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

	// send a command to set the channel
	atRequest = AtCommandRequest(CHCmd, CHSetVal, sizeof(CHSetVal));   
	STATUS |= sendAtCommand(atRequest) << 8;

	// send a command to set the baud rate
	atRequest = AtCommandRequest(BDCmd, BDSetVal, sizeof(BDSetVal));   
	STATUS |= sendAtCommand(atRequest) << 9;

	// send a command to set the power level
	atRequest = AtCommandRequest(PLCmd, PLSetVal, sizeof(PLSetVal));   
	STATUS |= sendAtCommand(atRequest) << 10;

	// return the status variable
	return STATUS;
}

void CCSDS_Xbee::printHex(int num, uint8_t precision) {
/*
prints the value with the specified number of digits in hex. will not
do anything if _debug_serial_defined (global variable) is set to false

Inputs: 
int num - The value to be printed
uint8_t precision - The number of digits to print

Outputs:
None

Return:
void

Globals:
bool _debug_serial_defined

example:
printHex(10, 2) would print '0x0A' to Serial0
printHex(1, 4) would print '0x0001' to Serial0

*/  

  if(precision > 8){
    precision = 8;
  }
  // initalize a buffer to hold the format spec
  // this buffer must be large enough to hold the format specifier for the sprint. In this case it need to hold '0x%.' + precision_len + 'X'
  char format[7];

  // form the format spec to print a hexidecimal value with a specified number of digits
  sprintf(format, "0x%%.%dX", precision);
    
  // initalize a buffer to hold the output string
  // this buffer must be large enough to hold the requested number of digits of precision + 2 (because it prefixed with '0x')
  char out_str[10];
    
  // form the output string to print
  sprintf(out_str, format, num);
    
  // write to Serial0
  Serial.print(out_str);

}

int CCSDS_Xbee::sendAtCommand(AtCommandRequest atRequest) {
/*
Function to send an AT command to the xbee and check for 
a response from the Xbee indicating command execution status.
Used to send commands to the xbee to configure it. Will print
debugging statements if _debug_serial_defined is true.

see reference for available AT commands:
http://examples.digi.com/wp-content/uploads/2012/07/XBee_ZB_ZigBee_AT_Commands.pdf

Inputs: 
AtCommandRequest atRequest - ATCommandRequest containg the command to be sent

Outputs:
None

Return:
flag indicating if the command was sent and acknowledged

Globals:
bool _debug_serial_defined

example:
  uint8_t PLCmd[] = {'P','L'};
  uint8_t PLSetVal[] = {0x04};

  STATUS |= sendAtCommand(AtCommandRequest(PLCmd, PLSetVal, sizeof(PLSetVal)));

*/	

	// initalize status variable to failure (will be changed if it passes)
	uint8_t STATUS = 1;

	if(_debug_serial_defined){
	// output a statement to indicate that a command is being sent
	Serial.println(F("Sending cmd to XBee:"));
	}
  
	// send the AT command which the user passed in
	xbee.send(atRequest);

	// once an AT command is sent we expect that the Xbee will send back 
	// an AT Command Response packet indicating if it executed or rejected
	// the command

	// wait up to 2 seconds for the response from the xbee
	// xbee.readpacket() returns true is a packet was read, false if it 
	//	timed out or there was an error reading from the xbee
	if (xbee.readPacket(1000)) {
	
		// if the response recieved is not an AT Command Response packet,
		//		tell the user and return a problem
		if (xbee.getResponse().getApiId() != AT_COMMAND_RESPONSE) {
			
			if(_debug_serial_defined){
			  Serial.print(F("Expected AT response but got "));
			  printHex(xbee.getResponse().getApiId(), 2);
			  Serial.println();
			}
			
			// return 1 to indicate that the command failed
			// FIXME: should we read again to see if the next packet is an AT response?
			return 1;
		  
		}
		// the response was an ATCommandResponse, process it
		else{
			// initalize a structure to hold the response from the xbee
			AtCommandResponse atResponse = AtCommandResponse();
		
			// get the ATCommandResponse from the xbee for further processing
			xbee.getResponse().getAtCommandResponse(atResponse);

			// check if the ATCommandResponse indicates that there was an error
			if (!atResponse.isOk()){
				
				if(_debug_serial_defined){
					// if there was an error, the Status contains the error code, print
					// 	it for the user
					Serial.print(F("Command return error code: "));
					printHex(atResponse.getStatus(), 2);
					Serial.println();
				}
				
				// return 1 to indicate the command failed 
				return 1;
			}
			// the command executed correctly
			else{
				
				if(_debug_serial_defined){
					// print the command sent to the debug
					Serial.print("Cmd [");
					Serial.print(char(atResponse.getCommand()[0]));
					Serial.print(char(atResponse.getCommand()[1]));
					Serial.print("] sent");
				
					// if the ATCommandResponse contained data
					if (atResponse.getValueLength() > 0) {
						
							// print the returned values
							Serial.print("Command value length is ");
							Serial.println(atResponse.getValueLength(), DEC);
							Serial.print(" and returned: ");
							for (int i = 0; i < atResponse.getValueLength(); i++) {
								printHex(atResponse.getValue()[i], 2);
								Serial.print(" ");
							}
							Serial.println();

					}
				}	
				
				// return 0 to indicate successful execution
				return 0;
			}
			
		}   
	} 
	// xbee.readpacket() returned a failure (could be either a timeout or error)
	else {
		
		// isError will return true if an error occured
		if (xbee.getResponse().isError()) {
			
			if(_debug_serial_defined){
				
				// print the error code for the user
				Serial.print(F("Error reading packet.  Error code: "));  
				printHex(xbee.getResponse().getErrorCode(),2);
				Serial.println();
			}
			
			// return 1 to indicate failure
			return 1;
		} 
		else {
			if(_debug_serial_defined){
				
				// tell the user that the call timed out
				Serial.println(F("Timeout, no response from radio")); 
			}		
			
			// return 1 to indicate failure
			return 1;
		}
	}
}


void CCSDS_Xbee::sendRawData(uint16_t SendAddr, uint8_t payload[], uint16_t payload_size){
/*

Function to send data to a remote xbee. Takes as arguments the address
of the remote xbee to send to, the data to be sent, and the size of the 
data to be sent.

Will send the data and print the SendCtr and the data sent to the serial.

Inputs: 
uint16_t SendAddr - the 16bit address to send the packet to
uint8_t payload[] - pointer to the array holding the bytes to be sent
int payload_size - number of bytes to send

Outputs:
None

Return:
None

Globals:
_debug_serial_defined
_SendCtr

example:
  sendRawData(0x0002, raw_data, 16);

*/	
  
	// create a TX16Request packet (ie, request that the xbee send a packet to a 16bit address)
	Tx16Request tx = Tx16Request(SendAddr, payload, payload_size);

	// Send the data.
	xbee.send(tx);
	// Note that the Xbee will automatically try to resend the packet 3 times if it 
	// does not receive and ACK.

	// update the counter keeping track of how many packets we've sent
	_SentPktCtr++;
  _SentByteCtr += payload_size;
  
  // log the packet
  logPkt(payload, payload_size, LOG_SENT);

	if(_debug_serial_defined){
		// Display data debug for the user
		_debug_serial->println();
		_debug_serial->print(F("Sent pkt #"));
		_debug_serial->print(_SentPktCtr);
		_debug_serial->print(F(", data: "));
		for(int i = 0; i < payload_size; i++){
			printHex(payload[i],2);
			_debug_serial->print(", ");
		}
		_debug_serial->println();
	}
}

int CCSDS_Xbee::createTlmMsg(uint8_t pkt_buf[], uint16_t _APID, uint8_t _payload[], uint16_t _payload_size){
	// if the user attempts to send a packet that's too long, return failure
	if(_payload_size + sizeof(CCSDS_TlmPkt_t) > PKT_MAX_LEN){
		if(_debug_serial_defined){
			Serial.println("Packet too long... not sending");
		}
		
		// return a failure
		return -1;
	}

  // clear the buffer
	//memset(pkt_buf, 0x00, pkt_buf+sizeof(CCSDS_TlmPkt_t));

	// fill primary header fields
	setAPID(pkt_buf, _APID);
	setSecHdrFlg(pkt_buf, 1);
	setPacketType(pkt_buf, CCSDS_TLM_PKT);
	setVer(pkt_buf, 0);
	setSeqCtr(pkt_buf, _SentPktCtr);
	setSeqFlg(pkt_buf, 0x03);
	setPacketLength(pkt_buf, _payload_size+sizeof(CCSDS_TlmPkt_t));

	// fill secondary header fields
#ifndef _NO_RTC_
	if(_rtc_defined){
		// get the current time from the RTC
    DateTime now = _rtc.now();
		// fill the fields
	  setTlmTimeSec(pkt_buf, now.unixtime());
	  setTlmTimeSubSec(pkt_buf, 0);
  }else{
#endif
	  setTlmTimeSec(pkt_buf, millis()/1000);
	  setTlmTimeSubSec(pkt_buf, millis()%1000);
#ifndef _NO_RTC_
	}
#endif

	// copy the packet data
	memcpy(pkt_buf+sizeof(CCSDS_TlmPkt_t), _payload, _payload_size);

	// update the payload_size to include the headers
	_payload_size += sizeof(CCSDS_TlmPkt_t);
	
	return _payload_size;
}

int CCSDS_Xbee::createCmdMsg(uint8_t pkt_buf[], uint16_t APID, uint8_t FcnCode, uint8_t payload[], uint16_t payload_size){
	// if the user attempts to send a packet that's too long, return failure
  
  // update the payload_size to include the headers
	payload_size += sizeof(CCSDS_CmdPkt_t);
  
  // make sure the packet won't be too long
	if(payload_size > PKT_MAX_LEN){
		if(_debug_serial_defined){
			Serial.println("Packet too long... not sending");
		}
		
		// return a failure
		return -1;
	}

	// fill primary header fields
	setAPID(pkt_buf, APID);
	setSecHdrFlg(pkt_buf, 1);
	setPacketType(pkt_buf, CCSDS_CMD_PKT);
	setVer(pkt_buf, 0);
	setSeqCtr(pkt_buf, _SentPktCtr);
	setSeqFlg(pkt_buf, 0x03);
	setPacketLength(pkt_buf, payload_size);

	// fill secondary header fields
	setCmdChecksum(pkt_buf, 0x0);
	setCmdFunctionCode(pkt_buf, FcnCode);

  // copy the packet data
	memcpy(pkt_buf+sizeof(CCSDS_CmdPkt_t), payload, payload_size);
  
	// write the checksum after the header's been added
	setCmdChecksum(pkt_buf, CCSDS_ComputeCheckSum((CCSDS_CmdPkt_t*) pkt_buf));
 
	return payload_size;
}


int CCSDS_Xbee::sendTlmMsg(uint16_t SendAddr, uint16_t APID, uint8_t payload[], uint16_t payload_size){
/*

Formats a CCSDS telemetry header and send the data to the indicated address.

Inputs: 
uint16_t SendAddr - the 16bit address to send the packet to
uint8_t payload[] - pointer to the array holding the bytes to be sent
int payload_size - number of bytes to send

Outputs:
None

Return:
Positive int is successful, negative int if failed

Globals:
_debug_serial_defined
_SendCtr

example:
  sendTlmMsg(0x0002, tlm_packet_data, 16);

*/	

  uint8_t pkt_buffer[PKT_MAX_LEN];
  uint8_t pkt_size = 0;
  
  pkt_size = createTlmMsg(pkt_buffer, APID, payload, payload_size);

	if(pkt_size > 0){
	
		// send the message
		sendRawData(SendAddr, pkt_buffer, pkt_size);

		// return successful execution
		return CCSDS_SUCCESS;
	}
	else{
		return CCSDS_FAILURE;
	}
}

  
int CCSDS_Xbee::sendCmdMsg(uint16_t SendAddr, uint16_t APID, uint8_t fcncode, uint8_t payload[], uint16_t payload_size){
/*

Formats a CCSDS command header and send the data to the indicated address.

Inputs: 
uint16_t SendAddr - the 16bit address to send the packet to
uint8_T fcncode - function code of the command to send
uint8_t payload[] - pointer to the array holding the bytes to be sent
int payload_size - number of bytes to send

Outputs:
None

Return:
Positive int is successful, negative int if failed

Globals:
_debug_serial_defined
_SendCtr

example:
  sendCmdMsg(0x0002, 0x01, cmd_packet_data, 4);

*/	

	uint8_t pkt_buffer[PKT_MAX_LEN];
  uint8_t pkt_size = 0;
  
  pkt_size = createCmdMsg(pkt_buffer, APID, fcncode, payload, payload_size);

	if(pkt_size > 0){
	
		// send the message
		sendRawData(SendAddr, pkt_buffer, pkt_size);

		// return successful execution
		return CCSDS_SUCCESS;
	}
	else{
		return CCSDS_FAILURE;
	}
}

int CCSDS_Xbee::readMsg(uint8_t packet_data[]){
  
  // call the version of readMsg with 0 timeout
	return readMsg(packet_data, 0);
}

int CCSDS_Xbee::readMsg(uint8_t packet_data[], uint16_t timeout){
/*

Extracts the payload and fcncode of a command message and returns it to the user. 

Inputs: 
uint16_t timeout - Time to wait for a packet before returning [ms]

Outputs:
None

Return:
Negative value if error occured, otherwise type (Cmd/Tlm) of packet received

example:
  uint16_t PktType = readMsg(1);

*/

  // read a message from the xbee
  return _readXbeeMsg(packet_data, timeout);
        
}


void CCSDS_Xbee::printPktInfo(CCSDS_PriHdr_t &_PriHeader){
/*

Prints debug info about a CCSDS packet. This function does nothing if _debug_serial_defined is false.

Inputs: 
CCSDS_PriHdr_t _PriHeader - Primary header of a packet

Outputs:
None

Return:
None

Globals:
_debug_serial_defined

example:
  printPktInfo(PriHeader);

*/
	
	if(_debug_serial_defined){
		
		// print info from the primary header
		_debug_serial->print("APID: ");
		_debug_serial->print(CCSDS_RD_APID(_PriHeader));
		_debug_serial->print(", SecHdr: ");
		_debug_serial->print(CCSDS_RD_SHDR(_PriHeader));
		_debug_serial->print(", Type: ");
		_debug_serial->print(CCSDS_RD_TYPE(_PriHeader));
		_debug_serial->print(", Ver: ");
		_debug_serial->print(CCSDS_RD_VERS(_PriHeader));
		_debug_serial->print(", SeqCnt: ");
		_debug_serial->print(CCSDS_RD_SEQ(_PriHeader));
		_debug_serial->print(", SegFlag: ");
		_debug_serial->print(CCSDS_RD_SEQFLG(_PriHeader));
		_debug_serial->print(", Len: ");
		_debug_serial->println(CCSDS_RD_LEN(_PriHeader));

		// process command and telemetry secondary headers
		if(CCSDS_RD_TYPE(_PriHeader)){

			// cast the data to a command secondary header
			CCSDS_CmdSecHdr_t _CmdSecHeader = *(CCSDS_CmdSecHdr_t*) (&_PriHeader+sizeof(CCSDS_PriHdr_t));

			// print the command-specific data
			_debug_serial->print("FcnCode: ");
			_debug_serial->print(CCSDS_RD_FC(_CmdSecHeader));
			_debug_serial->print(", CkSum: ");
			_debug_serial->println(CCSDS_RD_CHECKSUM(_CmdSecHeader));
		}
		else{

			// cast the data to a telemetry secondary header
			CCSDS_TlmSecHdr_t _TlmSecHeader = *(CCSDS_TlmSecHdr_t*) (&_PriHeader+sizeof(CCSDS_PriHdr_t));

			// print the telemetry-specific data
			_debug_serial->print("Sec: ");
			_debug_serial->print(CCSDS_RD_SEC_HDR_SEC(_TlmSecHeader));
			_debug_serial->print("Subsec: ");
			_debug_serial->println(CCSDS_RD_SEC_HDR_SUBSEC(_TlmSecHeader));
		}   
	}
}

int CCSDS_Xbee::_readXbeeMsg(uint8_t data[]){
  
  // call the other version with 0 timeout
	return _readXbeeMsg(data, 0);
}

int CCSDS_Xbee::_readXbeeMsg(uint8_t data[], uint16_t timeout){
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
    
  if(timeout == 0){
    // check if there's a message waiting
    xbee.readPacket();
  }
	else{
		// check if there's a message waiting, if not, keep checking until timeout
		// NOTE: readPacket doesn't work when the timeout is 0
		xbee.readPacket(timeout);
	}
  
  // check if a message was available
  if (!xbee.getResponse().isAvailable()){
    /*
    This typically means there was no packet to read, thogh can also mean
    that the xbee is hooked up correctly (though that should've been detected
    in the initalization
    */
    if(_debug_serial_defined){
      _debug_serial->println("No packet available");   
    }    
    return CCSDS_NO_RCVD;
  }
  
  // check if we read an error
  if (xbee.getResponse().isError()) {
    
    // tell the user that there was an error
    if(_debug_serial){
      _debug_serial->print(F("Read Error, Error Code:"));
      _debug_serial->println(xbee.getResponse().getErrorCode());
    }
    
    // return the error code
    return -xbee.getResponse().getErrorCode();
  }
  
  if(_debug_serial_defined){
    _debug_serial->println("Read pkt");
  }
  
  // if its a znet tx status            	
  if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
    if(_debug_serial_defined){
      _debug_serial->print(F("Received response..."));
    }
              
    // create response object for the message we're sending
    TxStatusResponse txStatus = TxStatusResponse();

    xbee.getResponse().getZBTxStatusResponse(txStatus);
    
    // get the delivery status, the fifth byte
    if (txStatus.getStatus() == SUCCESS) {
      // success.  time to celebrate
      if(_debug_serial_defined){
        _debug_serial->println(F("ACK!"));
      }
      return CCSDS_NO_RCVD;
    } 
    else{
     // the remote XBee did not receive our packet. sadface.
      if(_debug_serial_defined){
        _debug_serial->println(F("Not ACK"));
      }
      return CCSDS_NO_RCVD;
    }
  }
  else if( xbee.getResponse().getApiId() == RX_16_RESPONSE) {

    if(_debug_serial_defined){
      _debug_serial->println(F("Received Message..."));
    }
    
    // object for holding packets received over xbee
    Rx16Response reponse16 = Rx16Response();
    
    // read the packet
    xbee.getResponse().getRx16Response(reponse16);
      
    // copy data from packet into the data array starting at element 4
    memcpy(data, reponse16.getData(), reponse16.getDataLength());

    // record the RSSI from this packet
    this->_PrevPktRSSI = reponse16.getRssi();
    this->_PrevSenderAddr = reponse16.getRemoteAddress16();
      
#ifndef _NO_SD_
    if(_logfile_defined){
      // log the data as received
      logPkt(data, reponse16.getDataLength(), LOG_RCVD);
    }
#endif

    // record the new packet
    _RcvdPktCtr++;
    _RcvdByteCtr += reponse16.getDataLength();
    
    return reponse16.getDataLength();

  }
  else{
    if(_debug_serial_defined){
      _debug_serial->println(F("Received something else"));
    }
    return CCSDS_NO_RCVD;
  }   
    
}

#ifndef _NO_SD_
void CCSDS_Xbee::logPkt(uint8_t data[], uint8_t len, uint8_t received_flg){
  
  // call the version which logs to the local logfile
	logPkt(this->_logfile, data, len, received_flg);
}
#endif
	
void CCSDS_Xbee::logPkt(File logfile, uint8_t data[], uint8_t len, uint8_t received_flg){
/*  logPkt()
 * 
 *  Prints an entry in the given log file containing the given data. Will prepend an
 *  'S' if the data was sent or an 'R' is the data was received based on the value
 *  of the received_flg.
 */

  // if the file is open
  if (_logfile_defined && logfile) {

    // prepend an indicator of if the data was received or sent
    // R indicates this was received data
    if(received_flg == LOG_RCVD){
      logfile.print("R ");
    }
    if(received_flg == LOG_SENT){
      logfile.print("S ");
    }
    
    // print timestamp to file
    print_time(logfile);
    
    char buf[50];

    // print the data in hex
    for(int i = 0; i < len; i++){
        sprintf(buf, "%02x, ", data[i]);
        logfile.print(buf);
     }
     logfile.println();
     
     // ensure the data gets written to the file
     logfile.flush();
   }
}

void CCSDS_Xbee::print_time(File logfile){
/*  print_time()
 * 
 *  Prints the current time to the given log file. Time
 *  will always be of the format '0000, 00/00/00 00:00:00.000'
 *  where the first value is the value of millis and the
 *  second is the RTC value. Both values are always printed
 *  even if the RTC is not being used (it will be zeros)
 */
 
 // we always print the millis value
  logfile.print(millis());
  logfile.print(", ");
  
#ifndef _NO_RTC_
  if(_rtc_defined){
    // get the current time from the RTC
    //DateTime now = SoftRTC.now();
    DateTime now = _rtc.now();
    uint32_t nowMS = millis();

    // print a datestamp to the file
    char buf[50];
    //sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d.%03d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second(),(nowMS - start_millis)%1000);  // print milliseconds);
    sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d.%03d, ", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second(),millis()%1000);  // print milliseconds);
    logfile.print(buf);
  }
  else{
#endif
    logfile.print("00/00/00 00:00:00.000, ");
#ifndef _NO_RTC_
  }
#endif
  logfile.flush();
}

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

uint8_t validateChecksum(uint8_t _packet[]) {
	CCSDS_CmdPkt_t *header = (CCSDS_CmdPkt_t*) _packet;

	return CCSDS_ValidCheckSum(header);
}

CCSDS_PriHdr_t getPrimaryHeader(uint8_t _packet[]) {
	return *(CCSDS_PriHdr_t*)(_packet);
}

CCSDS_TlmSecHdr_t getTlmHeader(uint8_t _packet[]) {
	return *(CCSDS_TlmSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t)-1);
}

CCSDS_CmdSecHdr_t getCmdHeader(uint8_t _packet[]) {
	return *(CCSDS_CmdSecHdr_t*)(_packet+sizeof(CCSDS_PriHdr_t));
}

CCSDS_TlmPkt_t getTlmPkt(uint8_t _packet[]) {
	return *(CCSDS_TlmPkt_t*)(_packet);
}

CCSDS_CmdPkt_t getCmdPkt(uint8_t _packet[]) {
	return *(CCSDS_CmdPkt_t*)(_packet);
}
// FIXME: Doesnt work correctly
/*
uint16_t computeChecksum(uint8_t _packet[]) {
	return CCSDS_ComputeCheckSum(*(getCmdPkt(_packet)));
}
*/
uint8_t addStrToTlm(char *s, uint8_t payload[], uint8_t start_pos){

	memcpy(payload+start_pos,s,strlen(s));
  return start_pos + strlen(s);
  
}

/*
uint8_t addStrToTlm(const String &s, uint8_t payload[], uint8_t start_pos){
 // WARNING: Untested
	
  return addStrToTlm(s.c_str(), payload, start_pos);
  
}
*/

