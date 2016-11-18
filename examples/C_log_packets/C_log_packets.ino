/*
 * Arduino configuration for this sketch:
 * 
 * Arduino Mega
 * Xbee Rx/Tx attached to Serial3 Tx/Rx
 * SD card installed
 */
 
/* 
 *  This sketch builds upon simple_tlm_pkt to demonstrate how to sent and 
 *  received packets.
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

// Define the SD card chip select pin
#define CHIP_SELECT_PIN 53

// include the library
#include <SPI.h>
#include <SD.h>
#include "ccsds_xbee.h" // this always needs to be included after SD and RTC

// declare a CCSDS_Xbee object
CCSDS_Xbee ccsds_xbee;

// declare a file to log to
File xbeeLogFile;

// program variables
uint16_t cycle_ctr = 0;

void setup(){

  // Init Serials
  /*
   * Same as simple_tlm_msg
   */
  Serial.begin(250000);
  Serial3.begin(9600);
  
  //// Init Xbee
  /* 
   *  same as simple_tlm_msg
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
   * To start logging call start_logging with the oepn file
   * that you wish the log to be stored in. THE FILE MUST BE
   * OPEN BEFORE CALLING start_logging(). The logging will cease 
   * if the file is closed during the sketch. The library 
   * will flush its contents to the log after each write, 
   * the user does not need to close the file each loop.
   */
  ccsds_xbee.start_logging(xbeeLogFile);
  /*
   * If the user wanted to stop logging they would call 
   * 
   * ccsds_xbee.end_logging();
   * xbeeLogFile.close();
   * 
   * the user should not close the log file until after 
   * they've called end_logging(). This is the recommended
   * way of turning off logging.
   */
  
}
void loop(){

  // Initalize buffers/counters (same as simple_tlm_msg)
  uint8_t Pkt_Buff[PKT_MAX_LEN];
  uint8_t payload_buff[100];
  uint8_t payload_size = 0;
  uint8_t pktLength = 0;

  // Create telemetry packet (same as simple_tlm_msg)
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
  * Note, this is a different way determining if the packet sent from what
  * was used in simple_tlm_msg. Instead of comparing the sent packet counter
  * before/after sending the message, we can also examine the return value 
  * of sendTlmMsg.
  * 
  */ 
  if(ccsds_xbee.sendTlmMsg(0x0003, 0x00FF, payload_buff, payload_size)){
    Serial.print("Sent tlm packet,");
  }

 /* 
  *  The following code is from simple_rcv_cmd. Because logging is enabled
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
  else{
    Serial.println(" Message received!");

    /*
     *  In this sketch we don't process the received message, but 
     *  it would be done the same was as demonstrated in 
     *  simple_rcv_cmd
     */    
  }

  // increment the cycle counter
  cycle_ctr++;

  // wait a bit
  delay(1000);
  
}
