#include<Wire.h>
#include<XBee.h>
#include"ccsds_xbee.h"


void setup() {
  
  // send a message
  uint8_t tlm_data;
  addIntToTlm<uint8_t>((uint16_t)0xAA, &tlm_data, (uint16_t)0);
  sendTlmMsg((uint16_t)0x01, &tlm_data, (uint16_t)1);
}

void loop() {
  // put your main code here, to run repeatedly:

}
