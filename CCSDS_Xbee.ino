#include<Wire.h>
#include<XBee.h>
#include"ccsds_xbee.h"


void setup() {
  
  // send a message
  uint16_t pkt_pos = 0;
  uint8_t tlm_data;
  pkt_pos = addIntToTlm<uint8_t>((uint16_t)0xAA, &tlm_data, pkt_pos);
  pkt_pos = addStrToTlm("This is a test", &tlm_data, pkt_pos);
  sendTlmMsg((uint16_t)0x01, &tlm_data, pkt_pos);
  sendTlmMsg((uint16_t)0x01, (uint16_t)0x02, &tlm_data, pkt_pos);
}

void loop() {
  // put your main code here, to run repeatedly:

}
