/*
 * Arduino configuration for this sketch:
 * 
 * Arduino Mega
 * Xbee Rx/Tx attached to Serial3 Tx/Rx
 * 
 */
/*
 * Other configuration for this sketch:
 * 
 * Xbee attached to adafruit xbee breakout shield connected to XCTU
 * To send LED On Cmd from XCTU:
 *    In console view, load C_cmds_list_xctu
 *    Send LED_ON frame
 * To send LED Off Cmd from XCTU:
 *    If not already loaded, load B_cmds_list_xctu as above
 *    Send LED_OFF frame
 * The other frames defined in that file can be used to test
 * the various checks we're doing on the commands before they're
 * processed.
 */
 
// configure ccsds_xbee library
/* 
 *  comment same as B_simple_tlm_pkt
 */
#define _NO_RTC_

// include the library
#include "ccsds_xbee.h"

// declare a CCSDS_Xbee object
CCSDS_Xbee ccsds_xbee;

// program variables
uint16_t cycle_ctr = 0;

void setup(){

  // Init Serials
  /* 
   *  comment same as B_simple_tlm_pkt
   */
  Serial.begin(250000);
  Serial3.begin(9600);
  
  //// Init Xbee
  /* 
   *  comment same as B_simple_tlm_pkt
   */
  uint8_t initstat = ccsds_xbee.init(0x0002, 0x0B0B, Serial3, Serial);
  if(!initstat) {
    Serial.println("XBee Initialized!");
  } else {
    Serial.print("XBee Failed to Initialize with Error Code: ");
    Serial.println(initstat);
  }

  // initalize pin13 (the LED)
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop(){

  // create a buffer in which to compile the packet
  // make it longer than necessary
  uint8_t Pkt_Buff[PKT_MAX_LEN];
		
	// initalize a counter to keep track of packet length
	uint8_t pktLength = 0;

  /*
   * Calling readMsg will check if the xbee has received a message since 
   * the last check. If the xbee has received a message, readMsg() will 
   * retrieve it from the xbee and place the entire message in the buffer 
   * supplied in the first argument and return the number of bytes read. 
   * If no messages have been received by the xbee, readMsg will return 0. 
   * If an error occurs a negative value will be returned.
   * 
   * You may also call readMsg with a timeout by supplying an optional
   * second argument containing the number of millis seconds to wait 
   * for a message. SUPPLYING A SECOND ARGUMENT MADES READMSG A BLOCKING CALL.
   */ 
  uint8_t bytes_read = 0;
  bytes_read = ccsds_xbee.readMsg(Pkt_Buff);

  if(bytes_read < 0){
    Serial.print("Read failed with error code:");
    Serial.println(bytes_read);
  }
  else if(bytes_read == 0){
    Serial.println("No messages available");
  }
  else if(bytes_read != getPacketLength(Pkt_Buff)){
    /*
     * We use the getPacketLength function to extract the length field
     * of the packet header. If it doesn't match the number of bytes 
     * we actually received, we don't process it because we can't be 
     * sure if the packet was corrupted/truncated in transit or compiled 
     * wrong originally.
     */
     Serial.println("Received partial packet!");
  }
  else{
    Serial.println("Message received!");

    // call a function to process the packet
    packet_processing(Pkt_Buff);
  }

	// wait a bit
  /*
   * In general, you would probably not want to wait this long between reads
   * because the read function will read all packets available, which for long
   * times between reads, may be several. This example code is only set up to 
   * process one packet which begins at the beginning of the buffer, so all other
   * commands would be ignored.
   * 
   * For this example though, just don't send commands too quickly.
   */
  delay(1000);
  
}

void packet_processing(uint8_t Pkt_Buff[]){
   
   /*
   * APIDs define the type of packet, so we first check if the packet is 
   * the type we expect. For this example, we're expecting a command with a 
   * APID of 100. If the APID doesn't match then we stop processing this packet.
   */
  if(getAPID(Pkt_Buff) != 100){
    Serial.print("Invalid command received with APID  ");
    Serial.println(getAPID(Pkt_Buff));
    return;
  }
  /*
   * We then check to ensure that the packet we received is designated as a command.
   * Each APID should correspond to a type of packet... in this case we've defined 
   * APID 100 to be a command, but in theory there could be an APID 100 which is a command
   * and an APID 100 which is a telemetry packet (though its recommended that you don't 
   * reused APIDs at all, for clarity). If the packet isn't a command like
   * we expect it to be then we stop processing this packet.
   *
   * Note that if we wanted to check if a packet was a telemetry packet we could do so by
   * comparing the return value of getPacketType to CCSDS_TLM_PKT.
   */
  if(getPacketType(Pkt_Buff) != CCSDS_CMD_PKT){
    Serial.println("Packet recieved is not a command");
    return;
  }
  /*
   * Now that we've confirmed that the packet we received is a command, we know that
   * the primary header (which contains the APID and PktType flags we just checked) is
   * followed by a secondary command header, which contains a function code and a checksum.
   * The checksum of these packets is a hash generated from the contents of the packet. We
   * use it to verify that the data we received is what the sender intended. The function we're 
   * using here will extract the checksum included in the message, calculate our own checksum for
   * the data we received, and return true it matches the checksum sent with the packet or false
   * otherwise. If the packet's checksum doesn't validate then we stop processing this packet.
   */
  if(!packetHasValidChecksum(Pkt_Buff)){
    Serial.println("Command checksum doesn't validate");
    return;
  }

  /*
   * Now we can get to processing the actual content of the command. In this case, we've defined
   * 2 commands, one to turn the Arduino's LED on and another to turn it off. The commands are 
   * differentiated by their FcnCode, 1 for LED_ON and 2 for LED_OFF. Here, since the response
   * to the command is simple, we implement them in the logic, but more complicated functionality
   * could be broken out into functions.
   */
  switch(getCmdFunctionCode(Pkt_Buff)){
    case 1:{
      Serial.println("LED_ON command received");
      // turn on the LED
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    }
    case 2:{
      Serial.println("LED_OFF command received");
      // turn off the LED
      digitalWrite(LED_BUILTIN, LOW);  
      break;
    }
    default:{
      Serial.print("Command with unrecognized function code: ");
      Serial.println(getCmdFunctionCode(Pkt_Buff));
    }
  }
  
}