#include <stddef.h>
#include <kernel.h>
#include <stdlib.h>
#include <string.h>

//#include <util/json.h>
#include "mavlink/common/mavlink.h"
#include "mavlink/minimal/mavlink.h"
#include "PCLink.hpp"

#define SYSTEM_ID 20
#define DEFAULT_COMPONENT_ID MAV_COMP_ID_IMU

PCLink::PCLink(){
	mSerial = 0; 
}

void PCLink::SendHeartbeat(){
	if(!mSerial) return; 
	
	mavlink_system_t mavlink_system;
 
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
	 
	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	 
	uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
	 
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	 
	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	 
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	serial_putn(mSerial, buf, len);
}

void PCLink::SendRawIMU(uint64_t usec, 
		const glm::vec3 &acc, const glm::vec3 &gyr, const glm::vec3 &mag
){
	if(!mSerial) return; 
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[128];
	
	mavlink_msg_raw_imu_pack_chan(
		SYSTEM_ID, 
		DEFAULT_COMPONENT_ID, 
		MAVLINK_COMM_0,
		&msg,
		usec,
		acc.x, acc.y, acc.z, 
		gyr.x, gyr.y, gyr.z, 
		mag.x, mag.y, mag.z
	); 

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	serial_putn(mSerial, buf, len);
}

void PCLink::SendAttitude(uint32_t timestamp, 
	float roll, float pitch, float yaw, 
	float rate_roll, float rate_pitch, float rate_yaw
){
	if(!mSerial) return; 
	
	mavlink_attitude_t attmsg; 
	attmsg.time_boot_ms = timestamp; 
	attmsg.roll = glm::radians(roll); ///< Roll angle (rad, -pi..+pi)
	attmsg.pitch = glm::radians(pitch); ///< Pitch angle (rad, -pi..+pi)
	attmsg.yaw = glm::radians(yaw); ///< Yaw angle (rad, -pi..+pi)
	attmsg.rollspeed = glm::radians(rate_roll); ///< Roll angular speed (rad/s)
	attmsg.pitchspeed = glm::radians(rate_pitch); ///< Pitch angular speed (rad/s)
	attmsg.yawspeed = glm::radians(rate_yaw); ///< Yaw angular speed
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[100];
	 
	// Pack the message
	mavlink_msg_attitude_encode_chan(20, MAV_COMP_ID_IMU, 0, 
		&msg, &attmsg); 
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	serial_putn(mSerial, buf, len);
}

bool PCLink::ReceiveMessage(mavlink_message_t *msg){
	if(!mSerial) return false; 
	
	// Example variable, by declaring them static they're persistent
	// and will thus track the system state
	//static int packet_drops = 0;
	//static int mode = MAV_MODE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */

	mavlink_status_t status;
 
	// COMMUNICATION THROUGH EwhileXTERNAL UART PORT (XBee serial)
	
	uint8_t max_count = 16; 
	while(serial_waiting(mSerial) && (max_count--))
	{
		uint8_t c = serial_getc(mSerial);
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, msg, &status)) {
			return true; 
		}
	}
	// Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
	return false; 
}
