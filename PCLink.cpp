#include <stddef.h>
#include <kernel.h>
#include <stdlib.h>
#include <string.h>

//#include <util/json.h>
#include "PCLink.hpp"

#define SYSTEM_ID 20
#define DEFAULT_COMPONENT_ID MAV_COMP_ID_IMU

PCLink::PCLink(){
	mSerial = 0; 
}

void PCLink::SendHeartbeat(copter_state_t state){
	if(!mSerial) return; 
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_HEARTBEAT_LEN];
	 
	// Pack the message
	mavlink_msg_heartbeat_pack(
		20, MAV_COMP_ID_IMU, 
		&msg, 
		MAV_TYPE_QUADROTOR, 
		MAV_AUTOPILOT_GENERIC, 
		(state == COPTER_STATE_ACTIVE)?(MAV_MODE_STABILIZE_ARMED):MAV_MODE_PREFLIGHT, 
		0, 
		(MAV_STATE)state);// TODO: if mavlink changes then this will not work
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
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_RAW_IMU_LEN];
	
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

	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void PCLink::SendParamValueFloat(const char *name, float value, int count, int index){
	if(!mSerial) return; 
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_PARAM_VALUE_LEN];

	mavlink_msg_param_value_pack(
		20, MAV_COMP_ID_IMU, &msg,
		name, value, 
		MAV_PARAM_TYPE_REAL32,
		count, // count
		index+1 // index
	);

	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void PCLink::SendHud(
		float airspeed, 
		float groundspeed, 
		int16_t heading, 
		uint16_t throttle, 
		float alt, 
		float climb
){
	if(!mSerial) return; 
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_VFR_HUD_LEN];

	mavlink_msg_vfr_hud_pack(
		20, MAV_COMP_ID_IMU, 
		&msg,
		airspeed, 
		groundspeed, 
		heading, 
		throttle, 
		alt, 
		climb
	); 
	
	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void PCLink::SendAttitude(uint32_t timestamp, 
	float yaw, float pitch, float roll, 
	float rate_yaw, float rate_pitch, float rate_roll
){
	if(!mSerial) return; 
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_ATTITUDE_LEN];
	 
	// Pack the message
	mavlink_msg_attitude_pack(
		20, MAV_COMP_ID_IMU, 
		&msg, 
		timestamp,
		glm::radians(roll),
		glm::radians(pitch),
		glm::radians(yaw),
		glm::radians(rate_roll),
		glm::radians(rate_pitch),
		glm::radians(rate_yaw)
	); 
	
	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

bool PCLink::ReceiveMessage(mavlink_message_t *msg){
	if(!mSerial) return false; 
	
	mavlink_status_t status;
	
	uint8_t max_count = 16; 
	while(serial_waiting(mSerial) && (max_count--))
	{
		uint16_t c = serial_getc(mSerial);
		// Try to get a new message
		if(c != SERIAL_NO_DATA && mavlink_parse_char(MAVLINK_COMM_0, c, msg, &status)) {
			return true; 
		}
	}
	// Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
	return false; 
}
