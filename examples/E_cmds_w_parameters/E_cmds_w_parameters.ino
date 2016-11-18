/*
 * Arduino configuration for this sketch:
 * 
 * Arduino Mega
 * Xbee Rx/Tx attached to Serial3 Tx/Rx
 * SD card installed
 * RTC installed
 */
 
/* 
 *  This sketch builds upon D_rtc_logging to demonstrate how to send and 
 *  receive packets and using an RTC for logging. This sketch demonstrates
 *  the nominal usage of the library, so the configuration #defines present
 *  in the previous sketches are removed here.
 */

// Define the SD card chip select pin
#define CHIP_SELECT_PIN 53

// include the library
#include <SPI.h>
#include <SD.h>
#include "ccsds_xbee.h" // this always needs to be included after SD and RTC

// declare a CCSDS_Xbee object
CCSDS_Xbee ccsds_xbee;

// clear RTC objects
RTC_DS1307 rtc;  // real time clock (for logging with timestamps)

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
  SPI.begin();
  pinMode(CHIP_SELECT_PIN,OUTPUT);
  if (!SD.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD Card NOT detected.");
  }
  else{
    Serial.println("SD Card detected!");
  }
  
  //// Open log files
  xbeeLogFile = SD.open("XBEE_LOG.txt", FILE_WRITE);
  delay(10);

  /*
   * See example C_log_packets for addition info
   */
  ccsds_xbee.start_logging(xbeeLogFile);

 //// RTC  
  if (!rtc.begin()) {
    Serial.println("RTC NOT detected.");
  }
  else{
    Serial.println("RTC detected!");
  }

  /*
   * See example D_rtc_logging for addition info
   */
  ccsds_xbee.add_rtc(rtc);

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
  * Same as C_log_packetes
  */ 
  if(ccsds_xbee.sendTlmMsg(0x0003, 0x00FF, payload_buff, payload_size)){
    Serial.print("Sent tlm packet,");
  }

 /* 
  *  Same as C_log_packetes
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
     // don't process packets with incorrect lengths
     Serial.println("Received partial packet!");
  }
  else{
    Serial.println(" Message received!");

    // call a function to process the packet
    packet_processing(Pkt_Buff);
  }

  // increment the cycle counter
  cycle_ctr++;

  // wait a bit
  delay(1000);
  
}


void packet_processing(uint8_t Pkt_Buff[]){
 /* 
  *  Same as C_log_packets
  */
  if(getAPID(Pkt_Buff) == 100 && getPacketType(Pkt_Buff) == CCSDS_CMD_PKT && validateChecksum(Pkt_Buff)){

    /*
     * The type of command is identified by the function code included in 
     * the header. In this case we've defined 2 commands.
     */
    switch(getCmdFunctionCode(Pkt_Buff)){
      case 10:{
        /*  
         *   FcnCode 11 is defined as a command to allow the user to turn off
         *   and on the LED. The function is defined with the following format:
         *   
         *   CCSDS Command Header (8 bytes)
         *   OnOff flag (1 byte) (1 if on, 0 if off)
         */
        Serial.println("LED_ONOFF command received");

        uint8_t onoff_flag = 0;

        // extract the on_off_flag from the command
        /*
         * This demonstrates how to extract a value from a packet. This function
         * can be used with any integer or floating point type. The parameter to be
         * extracted is used as the first argument and that value will be modified to
         * contain the value from the packet. The packet data is passed as the second
         * argument, and the position of the parameter is the 3rd argument.
         * 
         * See the next command for another usage.
         */
        extractFromTlm(onoff_flag, Pkt_Buff, 8);

        // turn on the LED
        digitalWrite(LED_BUILTIN, onoff_flag);

      }
      case 11:{
        /*  
         *   FcnCode 11 is defined as a command to allow the user to set the
         *   cycle counter. The function is defined with the following format:
         *   
         *   CCSDS Command Header (8 bytes)
         *   CycleCtr flag (2 bytes)
         *   Param2 (1 bytes)
         *   Param3 (4 bytes)
         *   Param4 (4 bytes)
         */
        Serial.println("SetCtr command received");

        // create a counter to keep track of where in the packet we are
        /*
         * We initalize this to 8 so that we start reading at the end of 
         * the command header.
         */
        uint8_t pkt_pos = 8;
        uint8_t Param2 = 0;
        float Param3 = 0.0;
        uint32_t Param4 = 0;
        
        /*
         * The extractFromTlm function returns the new position of the 
         * packet after its extracted the current parameter. 
         */
         
        // extract the cycle_ctr value (uint16) from the command
        pkt_pos = extractFromTlm(cycle_ctr, Pkt_Buff, pkt_pos);
        Serial.print("With CycleCtr: ");
        Serial.println(cycle_ctr);
        
        // extract the Param2 value (uint8) from the command
        pkt_pos = extractFromTlm(Param2, Pkt_Buff, pkt_pos);
        Serial.print("With Param2: ");
        Serial.println(Param2);
        
        // extract the Param3 value (float) from the command
        pkt_pos = extractFromTlm(Param3, Pkt_Buff, pkt_pos);
        Serial.print("With Param3: ");
        Serial.println(Param3);

        // extract the Param4 value (uint32) from the command
        pkt_pos = extractFromTlm(Param4, Pkt_Buff, pkt_pos);
        Serial.print("With Param4: ");
        Serial.println(Param4);

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
