/*
This file contains code written to simplify the user interactions with 
the xbee and formating/recieving CCSDS packets.

Note: 
Variables and functions whose name begins with '_' are intended for use 
within this file only. Use in other code at your own risk.

Changelog:
SPL 	2016-06-28 	Added better commenting

*/

//////////////// include libraries
#include <XBee.h>

//////////////// include headers
// header file for this code 
#include "ccsds_xbee.h"
// defines the CCSDS packet format
#include "CCSDS.h"

//////////////// initalize objects
// initalize an Xbee object from the adafruit xbee library
XBee xbee = XBee();

//////////////// initalize variables

// flag controlling if debugging statements are printed to Serial0
// set to false by default, set by user by calling 4 argument InitXbee
bool _debug_serial = false;

// define the maximum expected length of packet's payload (ie data) to initalize buffer 
// FIXME: cite a source on the origin of this number. look in xbee documentation
#define PKT_MAX_LEN 100

// initalize buffer to hold packets for processing before its passed to the user
uint8_t _packet_data[PKT_MAX_LEN];

// initalize counter to hold number of bytes read
int _bytesread;

// initalize counters to track packet I/O
uint32_t _SendCtr = 0;
uint32_t _RcvdCtr = 0;
uint32_t _CmdRejCtr = 0;

void printHex(int num, uint8_t precision) {
/*
prints the value with the specified number of digits in hex. will not
do anything if _debug_serial (global variable) is set to false

Inputs: 
int num - The value to be printed
uint8_t precision - The number of digits to print

Outputs:
None

Return:
void

Globals:
bool _debug_serial

example:
printHex(10, 2) would print '0x0A' to Serial0
printHex(1, 4) would print '0x0001' to Serial0

*/  

  // if user has not suppressed serial output
  if(_debug_serial){
    
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
}

int sendAtCommand(AtCommandRequest atRequest) {
/*
Function to send an AT command to the xbee and check for 
a response from the Xbee indicating command execution status.
Used to send commands to the xbee to configure it. Will print
debugging statements if _debug_serial is true.

see reference for available AT commands:
http://examples.digi.com/wp-content/uploads/2012/07/XBee_ZB_ZigBee_AT_Commands.pdf

Inputs: 
AtCommandRequest atRequest - ATCommandRequest containg the command to be sent

Outputs:
None

Return:
flag indicating if the command was sent and acknowledged

Globals:
bool _debug_serial

example:
  uint8_t PLCmd[] = {'P','L'};
  uint8_t PLSetVal[] = {0x04};

  STATUS |= sendAtCommand(AtCommandRequest(PLCmd, PLSetVal, sizeof(PLSetVal)));

*/	

	// initalize status variable to failure (will be changed if it passes)
	uint8_t STATUS = 1;

	if(_debug_serial){
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
			
			if(_debug_serial){
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
				
				if(_debug_serial){
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
				
				if(_debug_serial){
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
			
			if(_debug_serial){
				
				// print the error code for the user
				Serial.print(F("Error reading packet.  Error code: "));  
				printHex(xbee.getResponse().getErrorCode(),2);
				Serial.println();
			}
			
			// return 1 to indicate failure
			return 1;
		} 
		else {
			if(_debug_serial){
				
				// tell the user that the call timed out
				Serial.println(F("Timeout, no response from radio")); 
			}		
			
			// return 1 to indicate failure
			return 1;
		}
	}
}

int InitXBee(uint16_t address, uint16_t PanID, Stream &xbee_serial, bool debug_serial){
/*
Initalizes the the xbee on xbee_serial and sets the following configuration parameters:
    Sets MY (16bit address) to the value given of the first argument (must be unique for each xbee)
    Sets ID (PAN ID) to to the value of the second argument (all Xbees much match)
    Sets CH (Channel) to C, arbitrarily chosen (all Xbees must match)
    Sets CE (Coordinator Enable), should be 0 for all xbees
    Sets PL (Power level), should be set at max
    Sets BD (Interface Data Rate) to 9600, must match arduino serial baud rate
    Sets AP (API enable) to enabled
	
These settings will allow for for packetized communication. Initalizing the xbee with this 
function will allow the user to specify if the library should print debug info to Serial0.

Inputs: 
uint16_t address - the 16bit address for the xbee (must be unqiue among xbees)
uint16_t PanID - the 16bit PanID for the xbee (must match other xbees)
Stream &xbee_serial - the serial port the xbee is attached to (Serial, Serial1, Serial2, Serial3)
bool debug_serial - a flag indicating if the library should print debug info the Serial0

Outputs:
None

Return:
bitfield indicating the status of the xbee configuration commands

Globals:
bool _debug_serial

example:
  InitXBee(0x0002, 0x0B0B, Serial3, true)

*/	

	// assign the flag indicating if the debug serial is being used
	_debug_serial = debug_serial;
	
	// call the normal init
	return InitXBee( address, PanID, xbee_serial);
	
}

int InitXBee(uint16_t address, uint16_t PanID, Stream &xbee_serial) {
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

void _sendData(uint16_t SendAddr, uint8_t payload[], uint16_t payload_size){
/*

INTENDED FOR INTERNAL USE ONLY

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
_debug_serial
_SendCtr

example:
  _sendData(0x0002, raw_data, 16);

*/	
  
	// create a TX16Request packet (ie, request that the xbee send a packet to a 16bit address)
	Tx16Request tx = Tx16Request(SendAddr, payload, payload_size);

	// Send the data.
	xbee.send(tx);
	// Note that the Xbee will automatically try to resend the packet 3 times if it 
	// does not receive and ACK.

	// update the counter keeping track of how many packets we've sent
	_SendCtr++;

	if(_debug_serial){
		// Display data debug for the user
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
}

int sendTlmMsg(uint16_t _SendAddr, uint8_t _payload[], uint16_t _payload_size){
  
  sendTlmMsg(_SendAddr, _SendAddr, _payload, _payload_size);
  
}

int sendTlmMsg(uint16_t _SendAddr, uint16_t _APID, uint8_t _payload[], uint16_t _payload_size){
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
_debug_serial
_SendCtr

example:
  sendTlmMsg(0x0002, tlm_packet_data, 16);

*/	

	// if the user attempts to send a packet that's too long, return failure
	if(_payload_size + sizeof(CCSDS_TlmPkt_t) > PKT_MAX_LEN){
		if(_debug_serial){
		  Serial.println("Packet too long... not sending");
		}
		
		// return a failure
		return -1;
	}

	// allocate the buffer to compile the packet in
	// buffer needs to be large enough to hold the user supplied data plus the headers
	uint8_t _packet_data[_payload_size+sizeof(CCSDS_TlmPkt_t)];
	memset(_packet_data, 0x00, _payload_size+sizeof(CCSDS_TlmPkt_t));

	// fill primary header fields
	setAPID(_packet_data, _APID);
	setSecHdrFlg(_packet_data, 1);
	setPacketType(_packet_data, 0);
	setVer(_packet_data, 0);
	setSeqCtr(_packet_data, _SendCtr);
	// FIXME: Make sure that 3 indicates a full packet
	// FIXME: Add multipacket support
	setSeqFlg(_packet_data, 0x03);
	setPacketLength(_packet_data, _payload_size+sizeof(CCSDS_TlmPkt_t));

	// fill secondary header fields
	setTlmTimeSec(_packet_data,millis()/1000L);
	setTlmTimeSubSec(_packet_data,millis() % 1000L);

	// copy the packet data
	memcpy(_packet_data+sizeof(CCSDS_TlmPkt_t), _payload, _payload_size);

	// update the payload_size to include the headers
	_payload_size += sizeof(CCSDS_TlmPkt_t);
	
	// send the message
	_sendData(_SendAddr, _packet_data, _payload_size);

	// return successful execution
	return 1;
}

int sendCmdMsg(uint16_t SendAddr, uint8_t fcncode, uint8_t payload[], uint16_t _payload_size){
  
  sendCmdMsg(SendAddr, SendAddr, fcncode, payload, _payload_size);
}
  
  
int sendCmdMsg(uint16_t SendAddr, uint16_t APID, uint8_t fcncode, uint8_t payload[], uint16_t _payload_size){
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
_debug_serial
_SendCtr

example:
  sendCmdMsg(0x0002, 0x01, cmd_packet_data, 4);

*/	

	// if the user attempts to send a packet that's too long, return failure
	if(_payload_size + sizeof(CCSDS_CmdPkt_t) > PKT_MAX_LEN){
		if(_debug_serial){
		  Serial.println("Packet too long... not sending");
		}

		// return a failure
		return -1;
	}

	// allocate the buffer to compile the packet in
	uint8_t _packet_data[_payload_size + sizeof(CCSDS_CmdPkt_t)];

	// fill primary header fields
	setAPID(_packet_data, APID);
	setSecHdrFlg(_packet_data, 1);
	setPacketType(_packet_data, 1);
	setVer(_packet_data, 0);
	setSeqCtr(_packet_data, _SendCtr);
	// FIXME: Make sure that 3 indicates a full packet
	// FIXME: Add multipacket support
	setSeqFlg(_packet_data, 0x03);
	setPacketLength(_packet_data, _payload_size+sizeof(CCSDS_CmdPkt_t));

	// fill secondary header fields
	setTlmTimeSec(_packet_data,millis()/1000L);
	setTlmTimeSubSec(_packet_data,millis() % 1000L);
	
	// fill secondary header fields
	setCmdChecksum(_packet_data, 0x0);
	setCmdFunctionCode(_packet_data, fcncode);

	// write the checksum after the header's been added
	setCmdChecksum(_packet_data, CCSDS_ComputeCheckSum((CCSDS_CmdPkt_t*) _packet_data));

	// copy the packet data
	memcpy(_packet_data+sizeof(CCSDS_CmdPkt_t), payload, _payload_size);

	// update the payload_size to include the headers
	_payload_size += sizeof(CCSDS_CmdPkt_t);
  
	// send the message
	_sendData(SendAddr, _packet_data, _payload_size);

	// return successful execution
	return 1;
}

int readMsg(uint16_t timeout){
/*

Extracts the payload and fcncode of a command message and returns it to the user. 

Inputs: 
uint16_t timeout - Time to wait for a packet before returning [ms]

Outputs:
None

Return:
Negative value if error occured, otherwise type (Cmd/Tlm) of packet received

Globals:
_debug_serial

example:
  uint16_t PktType = readMsg(1);

*/

  // read a message from the xbee
  _bytesread = _readXbeeMsg(_packet_data, timeout);
        
  if(_bytesread > 0){
    
    // return the packet type
    return (int)getPacketType(_packet_data);
  }
  else{
    // if an error occured, print it
    if(_bytesread < -1){
      if(_debug_serial){
        Serial.print("Error reading, code: ");
        Serial.print(_bytesread);
      }
    }
    return -1;
  }
}

int readCmdMsg(uint8_t params[], uint8_t &fcncode){
/*

Extracts the payload and fcncode of a command message and returns it to the user. 

Inputs: 
None

Outputs:
uint8_t params[]
uint8_t fcncode

Return:
Number of bytes read into data[]

Globals:
None

example:
	if(readMsg(1)){
		int lengthofparams = readCmdMsg(&params, &fcncode);
	}

*/
		
	// FIXME: handle case where it doesn't exist
	// FIXME: do we still want to return the data to the user if the header is missing?
	// FIXME: do we still want to return the data to the user if the checksum doesn't validate?
		
	// if the secondary header exists, extract the info from it
	if(getSecHdrFlg(_packet_data)){
		
		fcncode = getCmdFunctionCode(_packet_data);
	
		// FIXME: Add checksum checking and handling mismatched checksum
		/*
		if(CCSDS_ValidCheckSum ((CCSDS_CmdPkt_t*) (_packet_data))){
		Serial.println("Invalid");
		_CmdRejCtr++;
		return -1;
		}
		*/
	
	}
	else{
		
		// FIXME: indicate an error somehow
		fcncode = 0;

	}
	
	// copy the parameters into the user's pointer
	memcpy(params, _packet_data+sizeof(CCSDS_CmdPkt_t), _bytesread-sizeof(CCSDS_CmdPkt_t));

	// return param length
	return getPacketLength(_packet_data)-sizeof(CCSDS_CmdPkt_t);
	
}

int readTlmMsg(uint8_t data[]){
/*

Extracts the payload of a telemetry message and returns it to the user. 

Inputs: 
None

Outputs:
uint8_t data[]

Return:
Number of bytes read into data[]

Globals:
None

example:
	if(!readMsg(1)){
		int lengthofdata = readTlmMsg(data);
	}

*/

	// FIXME: should return time to user if they want it
	if(getSecHdrFlg(_packet_data)){
		//uint32_t time_sec = getTlmTimeSec(_packet_data);
		//uint16_t time_subsec = getTlmTimeSubSec(_packet_data);
	}

	// copy the parameters into the user's pointer
	memcpy(data, _packet_data+sizeof(CCSDS_TlmPkt_t), _bytesread-sizeof(CCSDS_TlmPkt_t));

	// return param length
	return getPacketLength(_packet_data)-sizeof(CCSDS_TlmPkt_t);
	
}

void printPktInfo(CCSDS_PriHdr_t &_PriHeader){
/*

Prints debug info about a CCSDS packet. This function does nothing if _debug_serial is false.

Inputs: 
CCSDS_PriHdr_t _PriHeader - Primary header of a packet

Outputs:
None

Return:
None

Globals:
_debug_serial

example:
  printPktInfo(PriHeader);

*/
	
	if(_debug_serial){
		
		// print info from the primary header
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

			// cast the data to a command secondary header
			CCSDS_CmdSecHdr_t _CmdSecHeader = *(CCSDS_CmdSecHdr_t*) (&_PriHeader+sizeof(CCSDS_PriHdr_t));

			// print the command-specific data
			Serial.print("FcnCode: ");
			Serial.print(CCSDS_RD_FC(_CmdSecHeader));
			Serial.print(", CkSum: ");
			Serial.println(CCSDS_RD_CHECKSUM(_CmdSecHeader));
		}
		else{

			// cast the data to a telemetry secondary header
			CCSDS_TlmSecHdr_t _TlmSecHeader = *(CCSDS_TlmSecHdr_t*) (&_PriHeader+sizeof(CCSDS_PriHdr_t));

			// print the telemetry-specific data
			Serial.print("Sec: ");
			Serial.print(CCSDS_RD_SEC_HDR_SEC(_TlmSecHeader));
			Serial.print("Subsec: ");
			Serial.println(CCSDS_RD_SEC_HDR_SUBSEC(_TlmSecHeader));
		}   
	}
}

int _readXbeeMsg(uint8_t data[], uint16_t timeout){
/*

INTENDED FOR INTERNAL USE ONLY

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
    
    if(_debug_serial){
      Serial.println("Read pkt ");
    }
      // if its a znet tx status            	
  	if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
          if(_debug_serial){
            Serial.print(F("Received response..."));
          }
                
      // create response object for the message we're sending
      TxStatusResponse txStatus = TxStatusResponse();

  	   xbee.getResponse().getZBTxStatusResponse(txStatus);
  		
  	   // get the delivery status, the fifth byte
         if (txStatus.getStatus() == SUCCESS) {
          	// success.  time to celebrate
              if(_debug_serial){
                Serial.print(F("ACK!"));
              }
              return 0;
         } else {
          	// the remote XBee did not receive our packet. sadface.
              if(_debug_serial){
                Serial.print(F("Not ACK"));
              }
                return 0;
         }
      }
     else if( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
       
      // record the new packet
      _RcvdCtr++;
      
      if(_debug_serial){
        Serial.print(F("Received Message..."));
      }
      
      // object for holding packets received over xbee
      Rx16Response reponse16 = Rx16Response();
      
      // read the packet
      xbee.getResponse().getRx16Response(reponse16);
        
      // copy data from packet into the data array starting at element 4
      memcpy(data, reponse16.getData(), reponse16.getDataLength());
      
      return reponse16.getDataLength();

     }
     else{
       if(_debug_serial){
        Serial.print(F("Received something else"));
       }
       return 0;
     }   
     
     if(_debug_serial){
      Serial.println();
     }
  } else if (xbee.getResponse().isError()) {
    
    // tell the user that there was an error
    if(_debug_serial){
      Serial.print(F("Read Error, Error Code:"));
      Serial.println(xbee.getResponse().getErrorCode());
    }
    
    // return the error code
    return -xbee.getResponse().getErrorCode();
    
  } else {
    /*
    This typically means there was no packet to read, thogh can also mean
    that the xbee is hooked up correctly (though that should've been detected
    in the initalization
    */
    if(_debug_serial){
      Serial.println("No packet available");   
    }    
    return -1;
  } 
}

uint8_t addStrToTlm(const char *s, uint8_t payload[], uint8_t start_pos){

	memcpy(&payload,s,strlen(s));
  return start_pos + strlen(s);
  
}