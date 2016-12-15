/*
 * Arduino configuration for this sketch:
 * 
 * Balloonduino
 * Xbee installed
 * SD card installed
 * RTC installed
 */
 
/* 
 *  This sketch builds upon the other tutorials, demonstrating the nominal 
 *  usage of the library for a balloon payload and defining the standard 
 *  commands that a payload implemented on a balloonduino is expected to 
 *  implement. It is designed to be  a starting point for payloads to 
 *  add their customizations / mission-specific functionality to.
 */
   
//// Includes:
#include <avr/wdt.h>  // watchdog timer
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_MCP9808.h"
#include "RTClib.h"  // RTC and SoftRTC
#include <Adafruit_BME280.h>
#include <SD.h>
#include <Adafruit_ADS1015.h>
#include "ccsds_xbee.h"
#include <SSC.h>
#include <EEPROM.h>

//// Enumerations
// APIDS
#define CMD_APID 200
#define HK_APID 210
#define ENV_APID 211
#define PWR_APID 212
#define IMU_APID 213
#define INIT_APID 214
#define TIME_APID 216
#define FILEINFO_APID 217
#define FILEPART_APID 218

// FcnCodes
#define NOOP_FCNCODE 0
#define REQHK_FCNCODE 10
#define REQENV_FCNCODE 11
#define REQPWR_FCNCODE 12
#define REQIMU_FCNCODE 13
#define REQINIT_FCNCODE 14
#define REQFLTR_FCNCODE 15
#define REQTIME_FCNCODE 16
#define REQFILEINFO_FCNCODE 17
#define REQFILEPART_FCNCODE 18
#define RESETCTR_FCNCODE 20
#define SETTIME_FCNCODE 21
#define REBOOT_FCNCODE 99

//// Declare objects
/*
 * These are standard addresses for a balloonduino and may need
 * to be adjusted for any custom electronics configuration being 
 * used by the payload. 
 */
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
RTC_DS1307 rtc;  // real time clock (for logging with timestamps)
RTC_Millis SoftRTC;   // This is the millis()-based software RTC
Adafruit_BME280 bme;
Adafruit_ADS1015 ads(0x4A);
SSC ssc(0x28, 255);
CCSDS_Xbee ccsds_xbee;

//// Serial object aliases
/* 
 * These are defined so that the user doesn't have to remember 
 * which serial (0, 1, 2, or 3) is hooked up to which device. 
 * Instead they call something like 
 *         debug_seral.println("this is a test"); 
 * so that its clear that's a debugging message and not something
 * else.
 */
#define debug_serial Serial
#define xbee_serial Serial3

//// Timing
// timing counters
uint16_t imu_read_ctr = 0;
uint16_t pwr_read_ctr = 0;
uint16_t env_read_ctr = 0;

// rate setting
// sensors will be read every X cycles
uint16_t imu_read_lim = 10;
uint16_t pwr_read_lim = 100;
uint16_t env_read_lim = 100;

//// Xbee setup parameters
/*
 * These setting should be set with agreement from the ground
 * system and all other payloads using xbees to avoid address
 * collisions. 
 */ 
uint16_t XBee_MY_Addr = 0x0002; // XBee address for this payload 
uint16_t XBee_PAN_ID = 0x0B0B; // XBee PAN address (must be the same for all xbees)

//// Data Structures
/*
 * Balloonduino has a number of built-in sensors which it uses to
 * collect engineering data about its operations and surroundings.
 * The following structures are used to organize the data being 
 * collected.
 */ 
// imu data
struct IMUData_s {
   uint8_t system_cal;
   uint8_t accel_cal;
   uint8_t gyro_cal;
   uint8_t mag_cal;
   float accel_x;
   float accel_y;
   float accel_z;
   float gyro_x;
   float gyro_y;
   float gyro_z;
   float mag_x;
   float mag_y;
   float mag_z;
}; 
// power data
struct PWRData_s {
  float batt_volt;
  float i_consump;
}; 
// environmental data
struct ENVData_s {
  float bme_pres;
  float bme_temp;
  float bme_humid;
  float ssc_pres;
  float ssc_temp;
  float bno_temp;
  float mcp_temp;
}; 
// environmental data
struct InitStat_s {
  uint8_t xbeeStatus;
  uint8_t rtc_running;
  uint8_t rtc_start;
  uint8_t BNO_init;
  uint8_t MCP_init;
  uint8_t BME_init;
  uint8_t SSC_init;
  uint8_t SD_detected;
}; 

//// Interface counters
/*
 * The following counters are used to keep track of how many 
 * commands are executed and rejected for use in debugging 
 * communication/commanding issues.
 */ 
uint16_t CmdExeCtr;
uint16_t CmdRejCtr;

//// Other variables
uint32_t start_millis = 0;
InitStat_s InitStat;

//// Files
/*
 * The data that balloonduino is generating is available for 
 * in-flight use and is also logged for post-processing and
 * determination of the environment during the flight. The following
 * files will log the data during the flight.
 */ 
// data logging files
File IMULogFile;
File ENVLogFile;
File PWRLogFile;
/*
 * These files log the data received/sent by this payload and the 
 * initialization state of the payload each time its powered on. The 
 * interface log is useful for debugging communication issues and the 
 * init log will record the status of each of balloonduino's sensors 
 * each time it starts up and can help in debugging data collected during
 * a flight.
 */ 
// interface logging files
File xbeeLogFile;
File initLogFile;

//// Function prototypes

// init functions
void openLogFiles();
void initalizeRTCandReturnStatus(struct InitStat_s *InitStat);
void initalizeSDandReturnStatus(struct InitStat_s *InitStat);
void initalizeXbeeAndReturnStatus(struct InitStat_s *InitStat);
void initalizeBNOandReturnStatus(struct InitStat_s *InitStat);
void initalizeMCPandReturnStatus(struct InitStat_s *InitStat);
void initalizeBMEandReturnStatus(struct InitStat_s *InitStat);
void initalizeSSCandReturnStatus(struct InitStat_s *InitStat);
void initalizeADSandReturnStatus(struct InitStat_s *InitStat);

// data handling
void respondToData(uint8_t data[], uint8_t data_len, struct IMUData_s IMUData, struct ENVData_s ENVData, struct PWRData_s PWRData);
void respondToCommand(uint8_t cmdData[], uint8_t cmdData_len, struct IMUData_s IMUData, struct ENVData_s ENVData, struct PWRData_s PWRData);
void respondToTelemetry(uint8_t data[], uint8_t data_len);

// pkt creation
uint16_t create_HK_payload(uint8_t Pkt_Buff[]);
uint16_t create_IMU_payload(uint8_t Pkt_Buff[], struct IMUData_s IMUData);
uint16_t create_PWR_payload(uint8_t Pkt_Buff[], struct PWRData_s PWRData);
uint16_t create_ENV_payload(uint8_t Pkt_Buff[], struct ENVData_s ENVData);
uint16_t create_INIT_payload(uint8_t Pkt_Buff[], struct InitStat_s InitStat);
uint16_t create_FileInfo_payload(uint8_t Pkt_Buff[], uint8_t file_idx);
uint16_t create_FilePart_payload(uint8_t Payload_Buff[], uint8_t file_idx, uint32_t start_pos, uint32_t end_pos);

// execute command
void executeReqHKCommand(uint8_t CmdData[]);
void executeNoOpCommand(uint8_t CmdData[]);
void executeResetCtrCommand(uint8_t CmdData[]);
void executeReqENVCommand(uint8_t CmdData[], struct ENVData_s ENVData);
void executeReqPWRCommand(uint8_t CmdData[], struct PWRData_s PWRData);
void executeReqIMUCommand(uint8_t CmdData[], struct IMUData_s IMUData);
void executeReqInitCommand(uint8_t CmdData[], struct InitStat_s InitData);
void executeSetTimeCommand(uint8_t CmdData[]);
void executeReqTimeCommand(uint8_t CmdData[]);
void executeReqFileInfoCommand(uint8_t CmdData[]);
void executeReqFilePartCommand(uint8_t CmdData[]);
void executeRebootCommand(uint8_t CmdData[]);

// sensor reading
void readImuSensorsIntoStruct(struct IMUData_s *IMUData);
void readPwrSensorsIntoStruct(struct PWRData_s *PWRData);
void readEnvSensorsIntoStruct(struct ENVData_s *ENVData);

// log data
void writeImuLogEntry(struct IMUData_s IMUData, File IMULogFile);
void writeEnvLogEntry(struct ENVData_s ENVData, File ENVLogFile);
void writePwrLogEntry(struct PWRData_s PWRData, File PWRLogFile);
void writeInitLogEntry(File initLogFile, struct InitStat_s InitStat);

// utility
void print_time(File file);
bool isTimeToReadImu(uint16_t imu_read_ctr);
bool isTimeToReadPwr(uint16_t pwr_read_ctr);
bool isTimeToReadEnv(uint16_t env_read_ctr);
bool isPacketCorrectLength(int BytesRead, uint8_t ReadData[]);

void setup() {
  /* setup()
   *  
   * Disables watchdog timer (in case its on)
   * Initalizes all the link hardware/software including:
   *   Serial
   *   Xbee
   *   RTC
   *   SoftRTC
   *   BNO
   *   MCP
   *   BME
   *   SSC
   *   ADS
   *   SD card
   *   Log files
   */

  // disable the watchdog timer immediately in case it was on because of a 
  // commanded reboot
  wdt_disable();

  //// Init serial ports:
  debug_serial.begin(250000);
  xbee_serial.begin(9600);

  debug_serial.println(F("Begin Init!"));
 
  initalizeRTCandReturnStatus(&InitStat);

  //// SoftRTC (for subsecond precision)
  SoftRTC.begin(rtc.now());  // Initialize SoftRTC to the current time
  start_millis = millis();  // get the current millisecond count

  initalizeSDandReturnStatus(&InitStat);
  
  openLogFiles();

  initalizeXbeeAndReturnStatus(&InitStat);
  
  initalizeBNOandReturnStatus(&InitStat);

  initalizeMCPandReturnStatus(&InitStat);

  initalizeBMEandReturnStatus(&InitStat);
  
  initalizeSSCandReturnStatus(&InitStat);
  
  initalizeADSandReturnStatus(&InitStat);

  //// set interface counters to zero
  CmdExeCtr = 0;
  CmdRejCtr = 0;
  
  writeInitLogEntry(initLogFile, InitStat);

  debug_serial.println(F("Initialized!"));
  
}

void loop() {
  /*  loop()
   *  
   *  Reads sensor data if cycle counters indicate to
   *  Reads from xbee and processes any data
   */
  
  // declare structures to store data
  /*
   * Note that these are local to the loop functions which means that
   * they are created each time loop starts and destroyed after it 
   * finishes. Since we read the sensors on a schedule, they will not
   * contain data each cycle.
   */ 
  IMUData_s IMUData;
  PWRData_s PWRData;
  ENVData_s ENVData;

  // increment read counters
  imu_read_ctr++;
  pwr_read_ctr++;
  env_read_ctr++;

  // read sensors if number of cycles since last read has elapsed
  if(isTimeToReadImu(imu_read_ctr)){
    readImuSensorsIntoStruct(&IMUData);
    writeImuLogEntry(IMUData, IMULogFile);
    imu_read_ctr = 0;
  }
  if(isTimeToReadPwr(pwr_read_ctr)){
    readPwrSensorsIntoStruct(&PWRData);
    writePwrLogEntry(PWRData, PWRLogFile);
    pwr_read_ctr = 0;
  }
  if(isTimeToReadEnv(env_read_ctr)){
    readEnvSensorsIntoStruct(&ENVData);
    writeEnvLogEntry(ENVData, ENVLogFile);
    env_read_ctr = 0;
  }  

  // initalize a counter to record how many bytes were read this iteration
  int BytesRead = 0;

  //// Read message from xbee
  // create a buffer to store the message
  uint8_t ReadData[100];

  // read the data from the xbee
  BytesRead = ccsds_xbee.readMsg(ReadData);

  // if data was read, process it as a CCSDS packet
  if(BytesRead > 0){
    
    if(isPacketCorrectLength(BytesRead, ReadData)){
      
      Serial.print(F("Received packet with expected length"));
      Serial.print(getPacketLength(ReadData));
      Serial.print(F(" but actual length "));
      Serial.print(BytesRead);
      
      // indicate that we rejected a command
      CmdRejCtr++;
    }
    else{

      // respond to the data
      respondToData(ReadData, BytesRead, IMUData, ENVData, PWRData);
    }
  }
  
  //// Payload-specific processing
  /* 
   * Implement any functionality that the payload needs to execute regularly
   * here. Any commands/responses will be implemented in command_response()
   */ 

  // wait for 10ms until we attempt to read more data
  /* NOTE: this introduces a limit on data throughput by how many packets can be processed
   *   per unit of time. With a 10ms delay, assuming 0 processing time (which is not realistic) 
   *   we can process at most 100 packets/sec.
   */
  delay(10);
}

void writeInitLogEntry(File initLogFile, struct InitStat_s InitStat){
  
  print_time(initLogFile);
  
  initLogFile.print(", ");
  initLogFile.print(InitStat.rtc_start);
  initLogFile.print(", ");
  initLogFile.print(InitStat.rtc_running);
  initLogFile.print(", ");
  initLogFile.print(InitStat.BNO_init);
  initLogFile.print(", ");
  initLogFile.print(InitStat.BME_init);
  initLogFile.print(", ");
  initLogFile.print(InitStat.MCP_init);
  initLogFile.print(", ");
  initLogFile.print(InitStat.SSC_init);
  initLogFile.print(", ");
  initLogFile.print(InitStat.xbeeStatus);
  initLogFile.print(", ");
  initLogFile.print(InitStat.SD_detected);
  initLogFile.println();
  initLogFile.close();
  
}

bool isTimeToReadImu(uint16_t imu_read_ctr){
  
  /*
   * There are several ways to decide when to read the IMU...
   * Either time-based sampling or cycle based cycling could 
   * be used. This function currently implements cycle-based
   * sampling but could be changed to trigger at a periodic 
   * time.
   */
  return imu_read_ctr > imu_read_lim;
}

bool isTimeToReadPwr(uint16_t pwr_read_ctr){
  return pwr_read_ctr > pwr_read_lim;
}

bool isTimeToReadEnv(uint16_t env_read_ctr){
  return env_read_ctr > env_read_lim;
}

bool isValidCommandHeader(uint8_t data[]){
  // get the APID (the field which identifies the type of packet)
  uint16_t APID = getAPID(data);
  
  /*
   * The following check confirms that the packet we received is
   * designated as a command packet.
   */ 
  if(!getPacketType(data)){
    debug_serial.print(F("Not a command packet"));
    return false;
  }
  
  /*
   * The following checks that the packet we received is a command 
   * for this payload to process.
   */ 
  if(APID != CMD_APID){
    debug_serial.print(F("Unrecognized apid 0x"));
    debug_serial.println(APID, HEX);
    return false;
  }
  
  /*
   * Commands contains a checksum which can be used to verify that
   * the packet we received is exactly what was sent. If it is not,
   * then we don't process the command because we can't be sure 
   * what parts of the messages might've been corrupted.
   */ 
  // validate command checksum
  if(!packetHasValidChecksum(data)){
    debug_serial.println(F("Command checksum doesn't validate"));
    CmdRejCtr++;
    return false;
  }
  
  return true;
}

bool isValidTelemetryHeader(uint8_t data[]){
 
  /*
   * The following check confirms that the packet we received is
   * designated as a telemetry packet.
   */ 
  if(getPacketType(data)){
    debug_serial.print(F("Not a telemetry packet"));
    return false;
  }
    
  return true;
}

void respondToData(uint8_t data[], uint8_t data_len, struct IMUData_s IMUData, struct ENVData_s ENVData, struct PWRData_s PWRData){
  
  if(isValidCommandHeader(data)){
    respondToCommand(data, data_len, IMUData, ENVData, PWRData);
  }
  
  if(isValidTelemetryHeader(data)){
    respondToTelemetry(data, data_len);
  }
}

void respondToTelemetry(uint8_t TlmData[], uint8_t data_len){
  
  // process each packet depending on what type of packet it is
  switch(getAPID(TlmData)){

    /* 
     * This code currently doesn't use any telemetry data it receives, so there 
     * are no packets processed by default. Examples of how it could be used include:
     *   - Setting payload time after sending a request for Link's flight time
     *   - Verifying that Link forwarded a message to the ground by examining
     *       Link's interface counters in HK telemetry, otherwise, resend 
     *   - Requesting a telemetry point from another payload (ex: request current
     *       flight position from another payload which is flying a GPS)
     *
     * In order to use a received packet, add a case to the switch statement to 
     * properly process the packet with the APID.
     */
     
    // unrecognized APID
    default:
    {
      debug_serial.print("Unrecognized telemetry packet with APID: 0x");
      debug_serial.println(getAPID(TlmData), HEX);
    }
  } // end switch(APID)
}

void respondToCommand(uint8_t CmdData[], uint8_t cmdData_len, struct IMUData_s IMUData, struct ENVData_s ENVData, struct PWRData_s PWRData) {
    
  /*
   * The function code determines what type of command it is and 
   * how to process it.
   */  
  switch(getCmdFunctionCode(CmdData)){

    // NoOp Cmd
    case NOOP_FCNCODE:
    {

      executeNoOpCommand(CmdData);
      break;
    }
    // REQ_HK Cmd
    case REQHK_FCNCODE:
    {
      
      executeReqHKCommand(CmdData);
      break;
    } 
    // ResetCtr
    case RESETCTR_FCNCODE:
    {
      
      executeResetCtrCommand(CmdData);
      break;
    }
    // ENV_Req
    case REQENV_FCNCODE:
    {
      
      executeReqENVCommand(CmdData, ENVData);
      break;
    }
    // PWR_Req
    case REQPWR_FCNCODE:
    {
      
      executeReqPWRCommand(CmdData, PWRData);
      break;
    }
    // IMU_Req
    case REQIMU_FCNCODE:
    {
      
      executeReqIMUCommand(CmdData, IMUData);
      break;
    }
    // Init_Req
    case REQINIT_FCNCODE:
    {
      
      executeReqInitCommand(CmdData, InitStat);
      break;
    }
    // SetTime
    case SETTIME_FCNCODE:
    {
      
      executeSetTimeCommand(CmdData);
      break;
    }
    // GetTime
    case REQTIME_FCNCODE:
    {
      
      executeReqTimeCommand(CmdData);
      break;
    }
    // Req_FileInfo
    case REQFILEINFO_FCNCODE:
    {
      
      executeReqFileInfoCommand(CmdData);
      break;
    }
    // Req_FilePart
    case REQFILEPART_FCNCODE:
    {
      
      executeReqFilePartCommand(CmdData);
      break;
    }
    // Reboot
    case REBOOT_FCNCODE:
    {
      
      executeRebootCommand(CmdData);
      break;    
    }
    /*
     * Payload Specific Commands
     *
     * This is where a payload will implement their specific commands and responses.
     */
     
    // unrecognized fcn code
    default:
    {
      debug_serial.print(F("unrecognized fcn code "));
      debug_serial.println(getCmdFunctionCode(CmdData), HEX);
      
      // reject command
      CmdRejCtr++;
    }
  } // end switch(FcnCode)

} // end command_response()

uint16_t create_HK_payload(uint8_t Pkt_Buff[]){
/*  create_HK_pkt()
 * 
 *  Creates an HK packet containing the values of all the interface counters. 
 *  Packet data is filled into the memory passed in as the argument. This function
 *  assumes that the buffer is large enough to hold this packet.
 *  
 */

  // initalize counter to record length of packet
  uint16_t payloadSize = 0;
  
  // Add counter values to the pkt
  payloadSize = addIntToTlm(CmdExeCtr, Pkt_Buff, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(CmdRejCtr, Pkt_Buff, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(ccsds_xbee.getRcvdByteCtr(), Pkt_Buff, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(ccsds_xbee.getSentByteCtr(), Pkt_Buff, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(millis()/1000L, Pkt_Buff, payloadSize); // Timer

  return payloadSize;
}

void print_time(File file){
/*  print_time()
 * 
 *  Prints the current time to the given log file
 */

  // get the current time from the RTC
  DateTime now = SoftRTC.now();
  uint32_t nowMS = millis();
  
  // print a datestamp to the file
  char buf[50];
  sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d.%03d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second(),(nowMS - start_millis)%1000);  // print milliseconds);
  file.print(buf);
}

void writeImuLogEntry(struct IMUData_s IMUData, File IMULogFile){
  
  // print the time to the file
  print_time(IMULogFile);

  // print the sensor values
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.system_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.mag_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_x);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_y);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_z);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_x);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_y);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_z);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.mag_x);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.mag_y);
  IMULogFile.print(", ");
  IMULogFile.println(IMUData.mag_z);

  IMULogFile.flush();
}
void writeEnvLogEntry(struct ENVData_s ENVData, File ENVLogFile){
/*  writeEnvLogEntry()
 * 
 *  Writes the ENV data to a log file with a timestamp.
 *  
 */
 
  // print the time to the file
  print_time(ENVLogFile);

  // print the sensor values
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bme_pres);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bme_temp);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bme_humid);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.ssc_pres);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.ssc_temp);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bno_temp);
  ENVLogFile.print(", ");
  ENVLogFile.println(ENVData.mcp_temp);

  ENVLogFile.flush();
}

void executeNoOpCommand(uint8_t CmdData[]){
  /*
   * This command requires no action other than to increment 
   * the command executed counter to indicate that it was received. 
   * This command is used to confirm communication with the payload
   * without causing any further action. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   * There are no parameters associated with this command.
   */ 
  debug_serial.println(F("Received NoOp Cmd"));

  // increment the cmd executed counter
  CmdExeCtr++;
}

void executeReqHKCommand(uint8_t CmdData[]){
  /* 
   * This command requests that a packet containing the payload's
   * interface information be sent to a specific xbee. The format of 
   * the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (uint8_t)
   * There is one parameter associated with this command, the address
   * of the xbee to send the HK message to. The format of the HK message
   * which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   Command Execution Counter (uint16_t)
   *   Command Rejection Counter (uint16_t)
   *   Received Bytes Counter (uint32_t)
   *   Sent Bytes Counter (uint32_t)
   *   Payload RunTime (uint32_t)
   */ 
  debug_serial.print(F("Received HKReq Cmd to addr "));

  // define variables to process the command
  uint8_t HKdestAddr = 0;
  uint16_t pktLength = 0;
  uint8_t payloadLength = 0;
  
  // define buffers to create the response in
  uint8_t HK_Payload_Buff[PKT_MAX_LEN];
  uint8_t HK_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desintation address from the command
  extractFromTlm(HKdestAddr, CmdData, 8);
  debug_serial.println(HKdestAddr);
  
  // add the information to the buffer
  payloadLength = create_HK_payload(HK_Payload_Buff);

  // create the telemetry message by adding the buffer to the header
  pktLength = ccsds_xbee.createTlmMsg(HK_Pkt_Buff, HK_APID, HK_Payload_Buff, payloadLength);
  
  if(pktLength > 0){
    // send the data
    ccsds_xbee.sendRawData(HKdestAddr, HK_Pkt_Buff, pktLength); 
  }

  // increment the cmd executed counter
  CmdExeCtr++;
}

void executeResetCtrCommand(uint8_t CmdData[]){
  /* 
   * This command requests that all of the payload's interface counters
   * be reset to zero. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   * There are no parameters associated with this command. 
   */ 
  debug_serial.println(F("Received ResetCtr Cmd"));

  // reset all counters to zero
  CmdExeCtr = 0;
  CmdRejCtr = 0;
  ccsds_xbee.resetCounters();

  // increment the cmd executed counter
  CmdExeCtr++;
}

void executeReqENVCommand(uint8_t CmdData[], struct ENVData_s ENVData){
  /* 
   * This command requests that the environmental status of the payload be
   * sent to a specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (uint8_t)
   * There is one parameter associated with this command, the address
   * of the xbee to send the ENV message to. The format of the ENV message
   * which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   BME pressure (float)
   *   BME temperature (float)
   *   BME humidity (float)
   *   SSC pressure (float)
   *   SSC temperature (float)
   *   BNO temperature (float)
   *   MCP temperature (float)
   */ 
  debug_serial.print(F("Received ENV_Req Cmd to addr: "));

  // define variables to process the command
  uint16_t EnvDestAddr = 0;
  uint16_t payloadLength = 0;
  int success_flg = 0;
  
  // define buffers to create the response in
  uint8_t ENV_Payload_Buff[PKT_MAX_LEN];
  uint8_t ENV_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desired xbee address from the packet
  extractFromTlm(EnvDestAddr, CmdData, 8);
  debug_serial.println(EnvDestAddr);

  // add the information to the buffer
  payloadLength = create_ENV_payload(ENV_Payload_Buff, ENVData);

  // send the telemetry message by adding the buffer to the header
  success_flg = ccsds_xbee.sendTlmMsg(EnvDestAddr, ENV_APID, ENV_Payload_Buff, payloadLength);
  
  if(success_flg > 0){
    // increment the cmd executed counter
    CmdExeCtr++;
  }
  else{
    CmdRejCtr++;
  }
}

void executeReqPWRCommand(uint8_t CmdData[], struct PWRData_s PWRData){
  /* 
   * This command requests that the power status of the payload be
   * sent to a specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (1 byte)
   * There is one parameter associated with this command, the address
   * of the xbee to send the PWR message to. The format of the PWR message
   * which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   Battery Voltage (float)
   *   Current consumption (float)
   */ 
  debug_serial.print(F("Received PWR_Req Cmd to addr: "));

  // define variables to process the command
  uint8_t PwrDestAddr = 0;
  uint8_t payloadLength = 0;
  int success_flg = 0;
  
  // define buffers to create the response in
  uint8_t PWR_Payload_Buff[PKT_MAX_LEN];
  uint8_t PWR_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desired xbee address from the packet
  extractFromTlm(PwrDestAddr, CmdData, 8);
  debug_serial.println(PwrDestAddr);
  
  // add the information to the buffer
  payloadLength = create_PWR_payload(PWR_Payload_Buff, PWRData);
  
  // send the telemetry message by adding the buffer to the header
  success_flg = ccsds_xbee.sendTlmMsg(PwrDestAddr, PWR_APID, PWR_Pkt_Buff, payloadLength);
  
  if(success_flg > 0){
    // increment the cmd executed counter
    CmdExeCtr++;
  }
  else{
    CmdRejCtr++;
  }
}

void executeReqIMUCommand(uint8_t CmdData[], struct IMUData_s IMUData){
  /* 
   * This command requests that the IMU status of the payload be
   * sent to a specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (1 byte)
   * There is one parameter associated with this command, the address
   * of the xbee to send the IMU message to. The format of the IMU message
   * which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   System Calibration (uint8_t)
   *   Accelerometer Calibration (uint8_t)
   *   Gyro Calibration (uint8_t)
   *   Magnetometer Calibration (uint8_t)
   *   Accelerometer X (float)
   *   Accelerometer Y (float)
   *   Accelerometer Z (float)
   *   Gyro X (float)
   *   Gyro Y(float)
   *   Gyro Z (float)
   *   Magnetometer X (float)
   *   Magnetometer Y (float)
   *   Magnetometer Z (float)
   */ 
  debug_serial.print(F("Received REQ_IMU Cmd to addr:"));

  // define variables to process the command
  uint8_t ImuDestAddr = 0;
  uint8_t payloadLength = 0;
  int success_flg = 0;
  
  // define buffers to create the response in
  uint8_t IMU_Payload_Buff[PKT_MAX_LEN];
  uint8_t IMU_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desired xbee address from the packet
  extractFromTlm(ImuDestAddr, CmdData, 8);
  debug_serial.println(ImuDestAddr);
  
  // add the information to the buffer
  payloadLength = create_IMU_payload(IMU_Payload_Buff, IMUData);
  
  // send the telemetry message by adding the buffer to the header
  success_flg = ccsds_xbee.sendTlmMsg(ImuDestAddr, IMU_APID, IMU_Pkt_Buff, payloadLength);
  
  if(success_flg > 0){
    // increment the cmd executed counter
    CmdExeCtr++;
  }
  else{
    CmdRejCtr++;
  }
}

void executeReqInitCommand(uint8_t CmdData[], struct InitStat_s InitStat){
  /* 
   * This command requests that the initalization status of the payload be
   * sent to a specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (1 byte)
   * There is one parameter associated with this command, the address
   * of the xbee to send the Init message to. The format of the INIT message
   * which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   Xbee stats (uint8_t)
   *   RTC Running (uint8_t)
   *   RTC Started (uint8_t)
   *   BNO Initalized (uint8_t)
   *   MCP Initalized (uint8_t)
   *   BME Initalized (uint8_t)
   *   SSC Initalized (uint8_t)
   *   SD Detected (uint8_t)
   */
  debug_serial.print(F("Received Req_Init Cmd to addr:"));

  // define variables to process the command
  uint8_t InitDestAddr = 0;
  uint8_t payloadLength = 0;
  int success_flg = 0;
  
  // define buffers to create the response in
  uint8_t Init_Payload_Buff[PKT_MAX_LEN];
  uint8_t Init_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desired xbee address from the packet
  extractFromTlm(InitDestAddr, CmdData, 8);
  debug_serial.println(InitDestAddr);
  
  // add the information to the buffer
  payloadLength = create_INIT_payload(Init_Payload_Buff, InitStat);
  
  // send the telemetry message by adding the buffer to the header
  success_flg = ccsds_xbee.sendTlmMsg(InitDestAddr, INIT_APID, Init_Payload_Buff, payloadLength);
  
  if(success_flg > 0){
    // increment the cmd executed counter
    CmdExeCtr++;
  }
  else{
    CmdRejCtr++;
  }
}

void executeSetTimeCommand(uint8_t CmdData[]){
  /* 
   * This command requests that the time of the RTC be set to the value 
   * included. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Time (4 byte)
   * There is one parameter associated with this command, the uint32_t 
   * number of seconds since unix epoch to set the RTC to.
   */
  debug_serial.print(F("Received SetTime Cmd with time: "));

  // define variables to process the command
  uint32_t settime = 0;
  
  // extract the time to set from the packet
  extractFromTlm(settime, CmdData, 8);
  debug_serial.println(settime);
  
  rtc.adjust(DateTime(settime));
  
  // increment the cmd executed counter
  CmdExeCtr++;
}

void executeReqTimeCommand(uint8_t CmdData[]){
  /* 
   * This command requests that the current RTC time of the payload be
   * sent to a specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (1 byte)
   * There is one parameter associated with this command, the address
   * of the xbee to send the time message to. The format of the TIME message
   * which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   Time (uint32_t)
   */
  debug_serial.print(F("Received Req_Time Cmd to addr:"));

  // define variables to process the command
  uint8_t TimeDestAddr = 0;
  uint8_t payloadLength = 0;
  int success_flg = 0;
  
  // define buffers to create the response in
  uint8_t Time_Payload_Buff[PKT_MAX_LEN];
  uint8_t Time_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desired xbee address from the packet
  extractFromTlm(TimeDestAddr, CmdData, 8);
  debug_serial.println(TimeDestAddr);

  // add the information to the buffer
  payloadLength = addIntToTlm(rtc.now().unixtime(), Time_Payload_Buff, payloadLength); // Add time to message [uint32_t]
  
  // send the telemetry message by adding the buffer to the header
  success_flg = ccsds_xbee.sendTlmMsg(TimeDestAddr, TIME_APID, Time_Pkt_Buff, payloadLength);
  
  if(success_flg > 0){
    // increment the cmd executed counter
    CmdExeCtr++;
  }
  else{
    CmdRejCtr++;
  }
}

void executeReqFileInfoCommand(uint8_t CmdData[]){
  /* 
   * This command requests that information about the specified file on the 
   * SD card be sent to a specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (1 byte)
   *   FileIdx (1 byte)
   * There are two parameters associated with this command, the address
   * of the xbee to send the FileInfo message to and the index of the file
   * on the SD card. The format of the FileInfo message which is sent out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   Filename (string, 8 bytes)
   *   Filesize (uint32_t)
   */
  debug_serial.print(F("Received Req_FileInfo Cmd to addr:"));

  // define variables to process the command
  uint8_t FileInfoDestAddr = 0;
  uint8_t payloadLength = 0;
  int success_flg = 0;
  uint8_t pkt_pos;
  uint8_t file_idx = 0;
  
  // define buffers to create the response in
  uint8_t FileInfo_Payload_Buff[PKT_MAX_LEN];
  uint8_t FileInfo_Pkt_Buff[PKT_MAX_LEN];
  
  // extract the desired xbee address from the packet
  pkt_pos = extractFromTlm(FileInfoDestAddr, CmdData, 8);
  debug_serial.print(FileInfoDestAddr);
  debug_serial.print(F(" for file at idx: "));

  // extract the index of the desired file from the packet
  pkt_pos = extractFromTlm(file_idx, CmdData, pkt_pos);
  debug_serial.println(file_idx);
  
  // create the packet payload
  payloadLength = create_FileInfo_payload(FileInfo_Payload_Buff, file_idx);
  
  // if we succeeded in creating the payload, create the packet
  if(payloadLength > 0){
    
    // send the telemetry message by adding the buffer to the header
    success_flg = ccsds_xbee.sendTlmMsg(FileInfoDestAddr, FILEINFO_APID, FileInfo_Pkt_Buff, payloadLength);
    
    if(success_flg > 0){
      // increment the cmd executed counter
      CmdExeCtr++;
    }
    else{
      CmdRejCtr++;
    }
  
  }
  else{
    CmdRejCtr++;
  }
}

void executeReqFilePartCommand(uint8_t CmdData[]){
  /* 
   * This command requests that part of a file on the SD card be sent to a 
   * specific xbee. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   *   Xbee address (1 byte)
   *   FileIdx (1 byte)
   *   Start Pos (4byte)
   *   End Pos (4byte)
   * There are four parameters associated with this command, the address
   * of the xbee to send the FilePart message, the index of the file
   * on the SD card, the starting byte position to send, and the ending byte
   * position to send. Start position must be less than end position and the 
   * difference in start position and end position must be smaller than 
   * the maximum packet size. The format of the FilePart message which is sent 
   * out is:
   *   CCSDS Telemetry Header (12 bytes)
   *   Filedata (uint8_t, variable length: end_pos-start_pos)
   */
  debug_serial.print(F("Received Req_FilePart Cmd to addr "));          
  
  
  // define variables to process the command
  uint8_t FilePartDestAddr = 0;
  uint8_t payloadLength = 0;
  int success_flg = 0;
  uint8_t pkt_pos;
  uint8_t file_idx = 0;
  uint32_t start_pos = 0;
  uint32_t end_pos = 0;
  
  // define buffers to create the response in
  uint8_t FilePart_Payload_Buff[PKT_MAX_LEN];
  uint8_t FilePart_Pkt_Buff[PKT_MAX_LEN];
    
  // extract the desired xbee address from the packet
  pkt_pos = extractFromTlm(FilePartDestAddr, CmdData, 8);
  debug_serial.print(FilePartDestAddr);
  debug_serial.print(F(" for file at idx: "));

  pkt_pos = extractFromTlm(file_idx, CmdData, pkt_pos);
  debug_serial.print(file_idx);

  debug_serial.print(F(" from pos: "));
  pkt_pos = extractFromTlm(start_pos, CmdData, pkt_pos);
  debug_serial.print(start_pos);
  debug_serial.print(F(" to: "));
  pkt_pos = extractFromTlm(end_pos, CmdData, pkt_pos);
  debug_serial.println(end_pos);

  // if the user requested a start position after the end 
  // position, then reject the command
  if(end_pos < start_pos){
    CmdRejCtr++;
    return;
  }
  
  // if the user requested more bytes than a packet can hold
  // then reject the command
  if(end_pos - start_pos > PKT_MAX_LEN - 12){
    CmdRejCtr++;
    return;
  }
  
  // create the packet payload
  payloadLength = create_FilePart_payload(FilePart_Payload_Buff, file_idx, start_pos, end_pos);
  
  // if we succeeded in creating the payload, create the packet
  if(payloadLength > 0){
      
    // send the telemetry message by adding the buffer to the header
    success_flg = ccsds_xbee.sendTlmMsg(FilePartDestAddr, FILEPART_APID, FilePart_Pkt_Buff, payloadLength);
    
    if(success_flg > 0){
      // increment the cmd executed counter
      CmdExeCtr++;
    }
    else{
      CmdRejCtr++;
    }
  }
  else{
    CmdRejCtr++;
  }
}

void executeRebootCommand(uint8_t CmdData[]){
  /* 
   * This command requests that the payload reboot itself. The format of the command is:
   *   CCSDS Command Header (8 bytes)
   * There are no parameters associated with this command.
   */
  debug_serial.println(F("Received Reboot Cmd"));

  // set the reboot timer to reboot in 1 second
  wdt_enable(WDTO_1S);

  // increment the cmd executed counter
  CmdExeCtr++;
}
      
void writePwrLogEntry(struct PWRData_s PWRData, File PWRLogFile){
/*  writePwrLogEntry()
 * 
 *  Writes the PWR data to a log file with a timestamp.
 *  
 */
 
  // print the time to the file
  print_time(PWRLogFile);
  
  // print the sensor values
  PWRLogFile.print(", ");
  PWRLogFile.print(PWRData.batt_volt,4);
  PWRLogFile.print(", ");
  PWRLogFile.println(PWRData.i_consump,4);

  PWRLogFile.flush();
}

void readEnvSensorsIntoStruct(struct ENVData_s *ENVData){
 
  //BME280
  ENVData->bme_pres = bme.readPressure() / 100.0F; // hPa
  ENVData->bme_temp = bme.readTemperature(); // degC
  ENVData->bme_humid = bme.readHumidity(); // %
/*
 * This is causing LINK to not respond to commands... not sure why
  //  SSC
  ssc.update();
  ENVData->ssc_pres = ssc.pressure(); // PSI
  ENVData->ssc_temp = ssc.temperature(); // degC
*/
  // BNO
  ENVData->bno_temp = bno.getTemp();
  
  //MCP9808
  ENVData->mcp_temp = tempsensor.readTempC(); // degC
}

void readPwrSensorsIntoStruct(struct PWRData_s *PWRData){

  PWRData->batt_volt = ((float)ads.readADC_SingleEnded(2)) * 0.002 * 3.0606; // V
  PWRData->i_consump = (((float)ads.readADC_SingleEnded(3)) * 0.002 - 2.5) * 10;
}

void readImuSensorsIntoStruct(struct IMUData_s *IMUData){

  uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // get measurements
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); // (values in uT, micro Teslas)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // (values in rps, radians per second)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // (values in m/s^2)

  // assign them into global variables
  IMUData->system_cal = system_cal;
  IMUData->accel_cal = accel_cal;
  IMUData->gyro_cal = gyro_cal;
  IMUData->mag_cal = mag_cal;
  IMUData->accel_x = accel.x();
  IMUData->accel_y = accel.y();
  IMUData->accel_z = accel.z();
  IMUData->gyro_x = gyro.x();
  IMUData->gyro_y = gyro.y();
  IMUData->gyro_z = gyro.z();
  IMUData->mag_x = mag.x();
  IMUData->mag_y = mag.y();
  IMUData->mag_z = mag.z();

}

uint16_t create_ENV_payload(uint8_t payload[], struct ENVData_s ENVData){
/*  create_ENV_pkt()
 * 
 *  Creates an ENV packet containing the values of all environmental sensors. 
 *  Packet data is filled into the memory passed in as the argument. This function
 *  assumes that the buffer is large enough to hold this packet.
 *  
 */

  uint16_t payloadSize = 0;
  // Add counter values to the pkt
  payloadSize = addFloatToTlm(ENVData.bme_pres, payload, payloadSize); // Add bme pressure to message [Float]
  payloadSize = addFloatToTlm(ENVData.bme_temp, payload, payloadSize); // Add bme temperature to message [Float]
  payloadSize = addFloatToTlm(ENVData.bme_humid, payload, payloadSize); // Add bme humidity to message [Float]
  payloadSize = addFloatToTlm(ENVData.ssc_pres, payload, payloadSize); // Add ssc pressure to message [Float]
  payloadSize = addFloatToTlm(ENVData.ssc_temp, payload, payloadSize); // Add ssc temperature to messsage [Float]
  payloadSize = addFloatToTlm(ENVData.bno_temp, payload, payloadSize); // Add bno temperature to message [Float]
  payloadSize = addFloatToTlm(ENVData.mcp_temp, payload, payloadSize); // Add mcp temperature to message [Float]

  return payloadSize;

}

uint16_t create_PWR_payload(uint8_t Pkt_Buff[], struct PWRData_s PWRData){
/*  create_PWR_pkt()
 * 
 *  Creates an PWR packet containing the values of all the power/battery sensors. 
 *  Packet data is filled into the memory passed in as the argument. This function
 *  assumes that the buffer is large enough to hold this packet.
 *  
 */
  
  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // Add counter values to the pkt
  payloadSize = addFloatToTlm(PWRData.batt_volt, Pkt_Buff, payloadSize); // Add battery voltage to message [Float]
  payloadSize = addFloatToTlm(PWRData.i_consump, Pkt_Buff, payloadSize); // Add current consumption to message [Float]

  return payloadSize;

}

uint16_t create_IMU_payload(uint8_t Pkt_Buff[], struct IMUData_s IMUData){
/*  create_IMU_pkt()
 * 
 *  Creates an IMU packet containing the values of all the IMU sensors. 
 *  Packet data is filled into the memory passed in as the argument. This function
 *  assumes that the buffer is large enough to hold this packet.
 *  
 */
  
  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // Add counter values to the pkt
  payloadSize = addIntToTlm(IMUData.system_cal, Pkt_Buff, payloadSize); // Add system cal status to message [uint8_t]
  payloadSize = addIntToTlm(IMUData.accel_cal, Pkt_Buff, payloadSize); // Add accelerometer cal status to message [uint8_t]
  payloadSize = addIntToTlm(IMUData.gyro_cal, Pkt_Buff, payloadSize); // Add gyro cal status to message [uint8_t]
  payloadSize = addIntToTlm(IMUData.mag_cal, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  payloadSize = addFloatToTlm(IMUData.accel_x, Pkt_Buff, payloadSize); // Add battery accelerometer x to message [Float]
  payloadSize = addFloatToTlm(IMUData.accel_y, Pkt_Buff, payloadSize); // Add battery accelerometer y to message [Float]
  payloadSize = addFloatToTlm(IMUData.accel_z, Pkt_Buff, payloadSize); // Add battery accelerometer z to message [Float]
  payloadSize = addFloatToTlm(IMUData.gyro_x, Pkt_Buff, payloadSize); // Add battery accelerometer x to message [Float]
  payloadSize = addFloatToTlm(IMUData.gyro_y, Pkt_Buff, payloadSize); // Add battery accelerometer y to message [Float]
  payloadSize = addFloatToTlm(IMUData.gyro_z, Pkt_Buff, payloadSize); // Add battery accelerometer z to message [Float]
  payloadSize = addFloatToTlm(IMUData.mag_x, Pkt_Buff, payloadSize); // Add battery accelerometer x to message [Float]
  payloadSize = addFloatToTlm(IMUData.mag_y, Pkt_Buff, payloadSize); // Add battery accelerometer y to message [Float]
  payloadSize = addFloatToTlm(IMUData.mag_z, Pkt_Buff, payloadSize); // Add battery accelerometer z to message [Float]
  
  return payloadSize;

}

uint16_t create_INIT_payload(uint8_t Pkt_Buff[], struct InitStat_s InitStat){
/*  create_IMU_pkt()
 * 
 *  Creates an IMU packet containing the values of all the IMU sensors. 
 *  Packet data is filled into the memory passed in as the argument. This function
 *  assumes that the buffer is large enough to hold this packet.
 *  
 */
  
  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // Add counter values to the pkt
  payloadSize = addIntToTlm(InitStat.xbeeStatus, Pkt_Buff, payloadSize); // Add system cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.rtc_running, Pkt_Buff, payloadSize); // Add accelerometer cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.rtc_start, Pkt_Buff, payloadSize); // Add gyro cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.BNO_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.MCP_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.BME_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.SSC_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  payloadSize = addIntToTlm(InitStat.SD_detected, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  
  return payloadSize;

}

//void send_and_log(uint8_t dest_addr, uint8_t data[], uint8_t data_len){
/*  send_and_log()
 * 
 *  If the destination address is 0, calls radio_send_and_log. Otherwise
 *  calls xbee_send_and_log
 *  
 */

    // send the HK packet via xbee and log it
//    ccsds_xbee.sendRawData(dest_addr, data, data_len);

//}


uint16_t create_FileInfo_payload(uint8_t Payload_Buff[], uint8_t file_idx){
  
  // declare file objects
  File rootdir;
  File entry;
  uint16_t payloadLength = 0;

  rootdir = SD.open("/");
  rootdir.seek(0);

  for(uint8_t i = 0; i < file_idx; i++){

    // open next file
    entry =  rootdir.openNextFile();
  }

  // if file idx exists
  if (entry && !entry.isDirectory()) {

      // Add counter values to the pkt
      char filename[13]; // max filename is 8 characters + 1 period + 3 letter extention + 1 null term
      sprintf(filename,"%12s",entry.name());

      for(int i = 0; i < 13; i++){
        debug_serial.print(filename[i]);
      }
      debug_serial.println();

      payloadLength = addStrToTlm(filename, Payload_Buff, payloadLength);
      payloadLength = addIntToTlm(entry.size(), Payload_Buff, payloadLength);
  }
  
  // close the files
  entry.close();
  rootdir.close();
      
  return payloadLength;
}


uint16_t create_FilePart_payload(uint8_t Payload_Buff[], uint8_t file_idx, uint32_t start_pos, uint32_t end_pos){
  
  // declare file objects
  File rootdir;
  File entry;
  uint16_t payloadLength = 0;

  rootdir = SD.open("/");
  rootdir.seek(0);

  for(uint8_t i = 0; i < file_idx; i++){

    // open next file
    entry =  rootdir.openNextFile();
    
  }

  // if file idx exists
  if (entry && !entry.isDirectory()) {

    // Add counter values to the pkt
    entry.seek(start_pos);

    // not sure why this doesn't work
    // error: invalid conversion from 'uint8_t {aka unsigned char}' to 'void*' [-fpermissive]
    //entry.read(Pkt_Buff[payloadSize], end_pos-start_pos);
    //payloadSize += end_pos-start_pos;

    for(int i = 0; i < end_pos-start_pos; i++){
      payloadLength = addIntToTlm(entry.read(), Payload_Buff, payloadLength);
    }

  }
  
  // close the files
  entry.close();
  rootdir.close();
  
  return payloadLength;
}

void initalizeRTCandReturnStatus(struct InitStat_s *InitStat){
  /* The RTC is used so that the log files contain timestamps. If the RTC
   *  is not running (because no battery is inserted) the RTC will be initalized
   *  to the time that this sketch was compiled at.
   */
  InitStat->rtc_start = rtc.begin();
  if (!InitStat->rtc_start) {
    debug_serial.println(F("RTC NOT detected."));
  }
  else{
    debug_serial.println(F("RTC detected!"));
    InitStat->rtc_running = rtc.isrunning();
  }
}

void initalizeSDandReturnStatus(struct InitStat_s *InitStat){
  //// Init SD card
  /* The SD card is used to store all of the log files.
   */
  SPI.begin();
  pinMode(53,OUTPUT);
  InitStat->SD_detected = SD.begin(53);
  if (!InitStat->SD_detected) {
    debug_serial.println(F("SD Card NOT detected."));
  }
  else{
    debug_serial.println(F("SD Card detected!"));
  }
}

void openLogFiles(){
  //// Open log files
  // NOTE: Filenames must be shorter than 8 characters
   
  xbeeLogFile = SD.open("XBEE_LOG.txt", FILE_WRITE);
  delay(10);
  
  // for data files, write a header
  initLogFile = SD.open("INIT_LOG.txt", FILE_WRITE);
  initLogFile.println(F("DateTime,RTCStart,RTCRun,BNO,BME,MCP,SSC,Xbee"));
  initLogFile.flush(); 
  delay(10);
  IMULogFile = SD.open("IMU_LOG.txt", FILE_WRITE);
IMULogFile.println(F("DateTime,SystemCal[0-3],AccelCal[0-3],GyroCal[0-3],MagCal[0-3],AccelX[m/s^2],AccelY[m/s^2],AccelZ[m/s^2],GyroX[rad/s],GyroY[rad/s],GyroZ[rad/s],MagX[uT],MagY[uT],MagZ[uT]"));
  IMULogFile.flush();  
  delay(10);
  PWRLogFile = SD.open("PWR_LOG.txt", FILE_WRITE);
  PWRLogFile.println(F("DateTime,BatteryVoltage[V],CurrentConsumption[A]"));
  PWRLogFile.flush();
  delay(10);
  ENVLogFile = SD.open("ENV_LOG.txt", FILE_WRITE);
  ENVLogFile.println(F("DateTime,BMEPressure[hPa],BMETemp[degC],BMEHumidity[%],SSCPressure[PSI],SSCTemp[degC],BNOTemp[degC],MCPTemp[degC]"));
  ENVLogFile.flush();
  delay(10);  
}

void initalizeXbeeAndReturnStatus(struct InitStat_s *InitStat){
  /* InitXbee() will configure the attached xbee so that it can talk to
   *   xbees which also use this library. It also handles the initalization
   *   of the adafruit xbee library
   */
  InitStat->xbeeStatus = ccsds_xbee.init(XBee_MY_Addr, XBee_PAN_ID, xbee_serial);
  if(!InitStat->xbeeStatus) {
    debug_serial.println(F("XBee Initialized!"));
  } else {
    debug_serial.print(F("XBee Failed to Initialize with Error Code: "));
    debug_serial.println(InitStat->xbeeStatus);
  }

  ccsds_xbee.add_rtc(rtc);
  ccsds_xbee.start_logging(xbeeLogFile);
}

void initalizeBNOandReturnStatus(struct InitStat_s *InitStat){
  //// BNO
  InitStat->BNO_init = bno.begin();
  if(!InitStat->BNO_init){
    debug_serial.println("BNO055 NOT detected.");
  }
  else{
    debug_serial.println("BNO055 detected!");
  }
  delay(200);
  bno.setExtCrystalUse(true);
}

void initalizeMCPandReturnStatus(struct InitStat_s *InitStat){
  //// MCP9808
  InitStat->MCP_init = tempsensor.begin(0x18);
  if (!InitStat->MCP_init) {
    debug_serial.println("MCP9808 NOT detected.");
  }
  else{
    debug_serial.println("MCP9808 detected!");
  }
}

void initalizeBMEandReturnStatus(struct InitStat_s *InitStat){
  //// Init BME
  // Temp/pressure/humidity sensor
  InitStat->BME_init = bme.begin(0x76);
  if (!InitStat->BME_init) {
    debug_serial.println("BME280 NOT detected.");
  }
  else{
    debug_serial.println("BME280 detected!");
  }
}

void initalizeSSCandReturnStatus(struct InitStat_s *InitStat){

  //// Init SSC
  //  set min / max reading and pressure, see datasheet for the values for your 
  //  sensor
  ssc.setMinRaw(0);
  ssc.setMaxRaw(16383);
  ssc.setMinPressure(0.0);
  ssc.setMaxPressure(30);
  //  start the sensor
  InitStat->SSC_init = ssc.start();
  if(!InitStat->SSC_init){
    debug_serial.println("SSC started ");
  }
  else{
    debug_serial.println("SSC failed!");
  }
}

void initalizeADSandReturnStatus(struct InitStat_s *InitStat){

  //// Init ADS
  // ADC, used for current consumption/battery voltage
  ads.begin();
  ads.setGain(GAIN_ONE);
  debug_serial.println("Initialized ADS1015");
}

bool isPacketCorrectLength(int BytesRead, uint8_t ReadData[]){
  return BytesRead != getPacketLength(ReadData);
}