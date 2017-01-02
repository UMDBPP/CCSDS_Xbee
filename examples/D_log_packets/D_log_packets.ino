/*
 * Arduino configuration for this sketch:
 * 
 * Arduino Mega
 * Xbee Rx/Tx attached to Serial3 Tx/Rx
 * SD card installed
 */
 
/* 
 *  This sketch builds upon B_simple_tlm_pkt and C_simple_rcv_cmd to demonstrate 
 *  how to send and received packets and have them be logged to a file.
 */

 
// configure ccsds_xbee library
/* 
 *  comment same as B_simple_tlm_pkt
 */
#define _NO_RTC_

// Define the SD card chip select pin
#define CHIP_SELECT_PIN 53

// include the library
/* 
 * Note that the ccsds_xbee library will include the SD and SPI libraries we
 * need to do the logging for us... they do not need to be included here (but
 * may be for clarity, if desired).
 */
#include "ccsds_xbee.h"

// declare a CCSDS_Xbee object
CCSDS_Xbee ccsds_xbee;

// declare a file to log to
File xbeeLogFile;

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

  //// Init SD card
  /* The SD card is used to store all of the log files.
   */
  SPI.begin();
  pinMode(CHIP_SELECT_PIN,OUTPUT);
  if (!SD.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD Card NOT detected.");
  }
  else{
    Serial.println("SD Card detected!");
  }
  
  //// Open log files
  /* 
   *  NOTE: The SD library requires that filenames must be 
   *  8 or fewer characters
   */
  xbeeLogFile = SD.open("XBEE_LOG.txt", FILE_WRITE);
  delay(10);

  /*
   * To start logging the I/O over the xbee call start_logging 
   * with the already opended file that you wish the log to be 
   * stored in. THE FILE MUST BE OPEN BEFORE CALLING start_logging().
   * The library will flush its contents to the log after each 
   * write. The user does not need to anything to the log file 
   * while the program is executing. The logging will cease if 
   * the log file is closed during the sketch. DO NOT CLOSE THE
   * FILE.
   */
  ccsds_xbee.start_logging(xbeeLogFile);
  /*
   * If the user wanted to stop logging (not recommended) they 
   * would call:
   * 
   * ccsds_xbee.end_logging();
   * xbeeLogFile.close();
   * 
   * The user should not close the log file until after 
   * they've called end_logging().
   */
  
}
void loop(){

  // Initalize buffers/counters (same as B_simple_tlm_pkt)
  uint8_t Pkt_Buff[PKT_MAX_LEN];
  uint8_t payload_buff[100];
  uint8_t payload_size = 0;
  uint8_t pktLength = 0;

  // Create telemetry packet (same as B_simple_tlm_pkt)
  payload_size = addIntToTlm(cycle_ctr, payload_buff, payload_size);
  uint32_t current_time_ms = millis();
  payload_size = addIntToTlm(current_time_ms, payload_buff, payload_size);
  float current_time_sec = current_time_ms / 1000.0;
  payload_size = addFloatToTlm(current_time_sec, payload_buff, payload_size);
  char sketch_name[] = "simple_tlm_pkt";
  payload_size = addStrToTlm(sketch_name, payload_buff, payload_size);
  
 /*
  * Now that logging is enabled the send function will automatically 
  * record the outgoing (and incoming) messages to the log file with a
  * timestamp.
  * 
  * Note, the code below is a different way determining if the packet 
  * was successfully sent from what was used in B_simple_tlm_msg. 
  * Instead of comparing the sent packet counter before/after sending 
  * the message, we can also examine the return value of sendTlmMsg. 
  *
  * Either way is acceptable. 
  */ 
  if(ccsds_xbee.sendTlmMsg(0x0003, 0x00FF, payload_buff, payload_size)){
    Serial.print("Sent tlm packet,");
  }

 /* 
  *  The following code is from C_simple_rcv_cmd. Because logging is enabled
  *  any messages received will also be logged to the same file.
  *  
  */
  uint8_t bytes_read = 0;
  bytes_read = ccsds_xbee.readMsg(Pkt_Buff);

  if(bytes_read < 0){
    Serial.print(" Read failed with error code:");
    Serial.println(bytes_read);
  }
  else if(bytes_read == 0){
    Serial.println(" No messages available");
  }
  else if(bytes_read != getPacketLength(Pkt_Buff)){
    /*
     * comment same as C_simple_rcv_cmd
     */
    Serial.println("Received partial packet!");
  }
  else{
    Serial.println(" Message received!");

    /*
     *  In this sketch we don't process the received message (because
     *  that's not important to demonstrating how logging works), but 
     *  it would be done the same was as demonstrated in 
     *  C_simple_rcv_cmd
     */    
  }

  // increment the cycle counter
  cycle_ctr++;

  // wait a bit
  delay(1000);
  
}