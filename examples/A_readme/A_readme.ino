/*
 * Just kidding... this isn't actually code. But this way the info you need shows up 
 * in the arduino IDE!
 * 
 * CCSDS_Xbee is a library which implements the CCSDS messaging protocol on an arduino 
 * via Xbee. Explaining those words:
 * 
 * CCSDS (Consultative Committee for Space Data Systems) is an organization of space 
 * agencies which have collaborated to create a standard messaging protocol which meets
 * the needs of typical spacecraft data management and communication. We've chosen to use
 * this protocol because our system has many of the same requirements as a spacecraft... 
 * the need to send verifiable commands to a remote system
 * the need to receive telemetry back from the remote system
 * the need to support routing of commands and telemetry through a system of multiple nodes
 * 
 * The protocol is implemented via the addition of a header to each packet of data moving through
 * the system. This header can have multiple parts. The most important part is called the primary 
 * header. This header contains the following fields:
 * 
 *   Application ID - ApID identifies the structure of the packet, unique to a specific type of packet
 *   Secondary Header Flag - Indicates if a secondary header is present (for our implementation, there 
 *     will always be a secondary header)
 *   Packet Type - Indicates is this packet is a command or telemetry packet
 *   Sequence Count - A counter which increments with each packet... used to reorder packets if they're
 *     recieved out of order
 *   Segmentation Flag - Indicates if this is a full or partial packet (for our implementation, the 
 *     the packets are always full packet)
 *   Packet Length - The length of the full packet (in bytes)
 * 
 * There will always be a secondary header following the primary header. There are two types, a 
 * command secondary header and a telemetry secondary header. The command secondary header contains
 * the following fields:
 * 
 *   Function Code - Indicates what type of command this is
 *   Checksum - A value used to verify that the packet received is the same as the packet which was 
 *     sent (ie, it wasn't corrupted in transit)
 *     
 * The telemetry secondary header contains the following fields:
 * 
 *   Timestamp - A timestamp associated with the data included in the packet.
 *   
 * The library takes care of handling most of these fields for you... the only user-relevant fields
 * of a packet are the ApID, FcnCode, and the data included in the packet. The APID is perhaps the 
 * most important field in the packet because it defines the format/structure of the packet. An APID
 * should be unique to a particular packet structure so that any entity which receives that packet
 * knows exactly how to process that packet (ie, the data types of the data in the packet, the order
 * they're in, etc) from the APID alone. 
 * 
 * An Xbee is a short-range bluetooth communication radio. They can be opporated in two modes, 
 * transparent mode and API mode. Transparent mode allows the user to write to the xbee's serial port 
 * and the data will be output by the xbee the user is paired with. While easy to use, this mode is 
 * limited because only two xbees can talk to eachother at once. The other mode, API mode, allows for 
 * packetized communication between any number of xbees. This is the mode that we use.
 * 
 * The xbee packet protocol is handled by adafruit's xbee library which handles packetizing and 
 * depacketizing the data for the user. The CCSDS_Xbee library and adafruit xbee library allow us to
 * use the following data flow:
 * 
 * 1) User specifies data 
 * 2) Header is added to data by ccsds_xbee library to form a CCSDS packet
 * 3) Header is added to CCSDS packet by xbee libary to form xbee packet
 * 4) Xbee packet is sent to local Xbee 
 * 5) Local Xbee sends xbee packet it received to remote xbee 
 * 6) Xbee packet is unwrapped by xbee library resulting in CCSDS packet
 * 7) CCSDS packet is unwrapped by ccsds_library resulting in original data
 * 
 * Steps 2-7 are designed to be transparent to the user. 
 * 
 * Xbees have a number of setting which must be correctly configured in order for them to talk with eachother. 
 * The configuration of the xbee is handled by ccsds_xbee each time the sketch starts during the init function. 
 * 
 * 
 */