/*
	This file is part of my quadcopter project.
	https://github.com/mkschreder/bettercopter

	This software is firmware project is free software: you can 
	redistribute it and/or modify it under the terms of the GNU General 
	Public License as published by the Free Software Foundation, either 
	version 3 of the License, or (at your option) any later version.

	This software is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

	Author: Martin K. Schröder
	Email: info@fortmax.se
	Github: https://github.com/mkschreder
*/

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


void PCLink::SendScaledIMU(uint64_t usec, 
		const glm::vec3 &acc, const glm::vec3 &gyr, const glm::vec3 &mag
){
	if(!mSerial) return; 
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_SCALED_IMU_LEN];
	
	mavlink_msg_scaled_imu_pack_chan(
		SYSTEM_ID, 
		DEFAULT_COMPONENT_ID, 
		MAVLINK_COMM_0,
		&msg,
		usec,
		acc.x * 1000, acc.y * 1000, acc.z * 1000, 
		gyr.x * 1000, gyr.y * 1000, gyr.z * 1000, 
		mag.x * 1000, mag.y * 1000, mag.z * 1000
	); 

	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void PCLink::SendMotorOutput(uint32_t timestamp, uint16_t front, uint16_t back, uint16_t left, uint16_t right){
	if(!mSerial) return; 
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN];

	mavlink_msg_servo_output_raw_pack(
		20, MAV_COMP_ID_IMU, &msg,
		timestamp, 0, 
		front, back, left, right, 
		0, 0, 0, 0); 

	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void PCLink::SendRCChannels(uint32_t timestamp, uint16_t chan1, uint16_t chan2, 
		uint16_t chan3, uint16_t chan4, uint16_t chan5, uint16_t chan6){
	if(!mSerial) return; 
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN];

	mavlink_msg_rc_channels_raw_pack(
		20, MAV_COMP_ID_IMU, &msg,
		timestamp, 0, //timestamp + chan
		chan1, chan2, chan3, chan4, chan5, chan6, 0, 0, 
		100 //rssi
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
		roll,
		pitch,
		yaw,
		rate_roll,
		rate_pitch,
		rate_yaw
	); 
	
	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void PCLink::SendPowerStatus(float vbat, float vcc){
	if(!mSerial) return; 
	
	if(vcc > 50) vcc = 50; if(vcc < 0) vcc = 0; 
	if(vbat > 50) vbat = 50; if(vbat < 0) vbat = 0; 
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_SYS_STATUS_LEN];
	
	mavlink_msg_sys_status_pack(
		20, //uint8_t system_id, 
		MAV_COMP_ID_IMU, //uint8_t component_id, 
		&msg, //mavlink_message_t* msg,
		MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_RC_RECEIVER, //uint32_t onboard_control_sensors_present, 
		MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_RC_RECEIVER, //uint32_t onboard_control_sensors_enabled, 
		MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_RC_RECEIVER, //uint32_t onboard_control_sensors_health, 
		3, //uint16_t load, 
		(uint16_t)(vbat * 1000), //uint16_t voltage_battery, 
		30, //int16_t current_battery, 
		45, //int8_t battery_remaining, 
		5, //uint16_t drop_rate_comm, 
		6, //uint16_t errors_comm, 
		7, //uint16_t errors_count1, 
		8, //uint16_t errors_count2, 
		9, //uint16_t errors_count3, 
		10 //uint16_t errors_count4
	); 
	serial_putn(mSerial, buf, mavlink_msg_to_send_buffer(buf, &msg));
	// Pack the message
	mavlink_msg_power_status_pack(
		20, MAV_COMP_ID_IMU, 
		&msg, 
		(uint16_t)(vbat * 1000), 
		(uint16_t)(vcc * 1000),
		MAV_POWER_STATUS_BRICK_VALID | MAV_POWER_STATUS_SERVO_VALID | 
			MAV_POWER_STATUS_CHANGED
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
