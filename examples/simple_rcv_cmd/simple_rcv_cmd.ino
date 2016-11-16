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
  
}
void loop(){

  // create a buffer in which to compile the packet
  // make it longer than necessary
  uint8_t Pkt_Buff[PKT_MAX_LEN];

  // create a buffer in which to compile the payload of the packet
  uint8_t payload_buff[100];
	
	// initalize a counter to keep track of the length of the payload
	uint8_t payload_size = 0;
	
	// initalize a counter to keep track of packet length
	uint8_t pktLength = 0;

	/*
	 * The code below compiles a telemetry packet by sequentially adding 
	 * telemetry points to the buffer using the provided helper functions.
	 * The available helper functions are:
	 *    addIntToTlm
	 *    addFloatToTlm
	 *    addStrToTlm
	 *  Each of these functions take the value to be added as the first 
	 * argument, the buffer to add it to in the second argument, and the
	 * position to add the point in the buffer as the third argument and 
	 * return the new position (with the telemetry point added). These 
	 * functions can be chained together in any manner as shown below.
	 */
	
  // add the cycle counter to telemetry
	payload_size = addIntToTlm(cycle_ctr, payload_buff, payload_size);
	
	// add the current processor time (in milliseconds) to telemetry
	uint32_t current_time_ms = millis();
	payload_size = addIntToTlm(current_time_ms, payload_buff, payload_size);
	
	// add the current processor time (in sec) to telemetry
	float current_time_sec = current_time_ms / 1000.0;
	payload_size = addFloatToTlm(current_time_sec, payload_buff, payload_size);
	
	// add the sketch name to telemetry
	char sketch_name[] = "simple_tlm_pkt";
	payload_size = addStrToTlm(sketch_name, payload_buff, payload_size);
	
	// record how many packets have been sent before we send a new one
	uint32_t pre_send = ccsds_xbee.getSentPktCtr();
	
	/*
	 * The code below creates the packet's header, appends the payload, and sends
	 * the packet. Each packet is identified by an APID (Application ID), which is 
	 * the second argument to the createTlmMsg function below. For this example
	 * we've defined the APID as 0xFF (255).
	 *
	 * The first argument to sendTlmMsg is the xbee address to send the message to,
	 * the second argument is the APID, the third is the buffer containing the packet's
	 * payload, and the third is the length of the payload. createTlmMsg returns 
	 * the total length of the packet (header+payload) or a negative value if an 
	 * error occured.
	 */ 
	ccsds_xbee.sendTlmMsg(0x0003, 0x00FF, payload_buff, payload_size);
	
	// print a message indicating how many packets we sent
	Serial.print("Sent ");
	Serial.print(ccsds_xbee.getSentPktCtr()-pre_send);
	Serial.println(" packets");

	// increment the cycle counter
  cycle_ctr++;

	// wait a bit
  delay(1000);
  
}