/*
 * Arduino configuration for this sketch:
 * 
 * Arduino Mega
 * Xbee Rx/Tx attached to Serial3 Tx/Rx
 * 
 */

// configure ccsds_xbee library
/*
 * This statement configures the CCSDS_Xbee library such that it does not 
 * attempt to load the RTC library, which the user may not have installed
 * (since its not a default library).
 */
#define _NO_RTC_
/*
 * This statement defines the maximum packet length the user intends to 
 * use. If the user does not define PKT_MAX_LEN CCSDS_Xbee will define
 * a default value which the user can use to initalize buffers.
 */
#define PKT_MAX_LEN 100

// include the library
#include "ccsds_xbee.h"

// declare a CCSDS_Xbee object
CCSDS_Xbee ccsds_xbee;

// program variables
uint16_t cycle_ctr = 0;

void setup(){

  // Init Serials
  /*
   * Note that the xbee's have only been tested at 9600baud at this time and
   * so the serial that the xbee is using must be set to this baud rate.
   */
  Serial.begin(250000);
  Serial3.begin(9600);
  
  //// Init Xbee
  /* InitXbee() will configure the attached xbee so that it can talk to
   *   xbees which also use this library. It also handles the initalization
   *   of the adafruit xbee library. 
   *   
   *   The arguments passed to init() are:
   *   init(uint16_t MY_Addr, uint16_t PanID, Stream &xbee_serial)
   *   
   *   MY_Addr is the address of this xbee. All xbee addresses must be 
   *   unqiue. The PanID defines the network for the xbee. All xbees must
   *   be on the same network in order to talk to eachother. The xbee_serial
   *   is the arduino Serial object that the Tx/Rx of the Xbee are connected 
   *   to. Then talking to the xbee the library will write to and read from
   *   that serial.
   */
  uint8_t initstat = ccsds_xbee.init(0x0002, 0x0B0B, Serial3);
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
   * Calling readMsg will check if the xbee has received a message. If
   * it has, it will place the entire message in the buffer supplied in
   * the first argument and return the number of bytes read. If not, it 
   * will return 0. If an error occurs a negative value will be returned.
   * 
   * You may also call readMsg with a timeout by supplying an optional
   * second parameter containing the number of millis seconds to wait 
   * for a message. THIS IS A BLOCKING CALL.
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
     * sure if the packet was corrupted/truncated in transit of compiled 
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
   * the type we expect. For the purposes of this example, lets say that
   * this payload expects an APID of 100 to contain a command. We check if
   * the packet has an APID of 100, if the packet is a command packet, and 
   * if the checksum is correct. If so we process it as a command.
   * 
   * In the CCSDS standard all packets contain a primary header which contain
   * the APID and the PktType flag. By determining its a command packet, we then
   * know that following the primary header will be a command secondary header 
   * which includes a checksum and the FcnCode. We've defined these commands to 
   * have no parameters, so that's all the information we need to process the 
   * command.
   */
  if(getAPID(Pkt_Buff) == 100 && getPacketType(Pkt_Buff) == CCSDS_CMD_PKT && validateChecksum(Pkt_Buff)){

    /*
     * The type of command is identified by the function code included in 
     * the header. In this case we've defined 2 commands.
     */
    switch(getCmdFunctionCode(Pkt_Buff)){
      case 1:{
        Serial.println("LED_ON command received");
        // turn on the LED
        digitalWrite(LED_BUILTIN, HIGH);
      }
      case 2:{
        Serial.println("LED_OFF command received");
        // turn off the LED
        digitalWrite(LED_BUILTIN, LOW);  
      }
      default:{
        Serial.println("Command with unrecognized function code received");
      }
    }
  }
  else{
    Serial.println("Invalid command received");
  }
    
}
