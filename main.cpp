/**
	This file is part of martink project.

	martink firmware project is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	martink firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

	Author: Martin K. Schröder
	Email: info@fortmax.se
	Github: https://github.com/mkschreder
*/
/*
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <boards/multiwii.h>*/

#include <kernel.h>
#include <disp/ssd1306.h>

#ifdef CONFIG_SIMULATOR
#include "simulator/sim_kernel.h"
#endif

/*
#include <net/tcpip.h>
#include <net/rfnet.h>*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "Application.hpp"
#include "FlightController.hpp"
//#include "PCLink.hpp"

#include "mavlink.h"

#define TXRX_NUM_CHANNELS 10
#define DEVICE_TWI_ADDRESS 0x88

//static struct uart uart;


void sim_init(); 
void sim_loop(); 

static glm::vec3 ofs; 

class Application {
public:
	Application() {}
	void init();
	void loop();
	
public: 
	bool PCLinkReceiveMessage(mavlink_message_t *msg); 
	void PCLinkSendHeartbeat(); 
	void PCLinkProcessEvents(); 
	
	void WriteConfig(const AppConfig &conf); 
	void ReadConfig(AppConfig &conf); 
private:
	struct fc_quad_interface hardware; 
	uint8_t armed, arm_progress; 
	timestamp_t arm_timeout; 
	FlightController fc; 
	serial_dev_t mPCSerial; 
};

static Application app;

void Application::init(){
#ifdef CONFIG_SIMULATOR
	sim_init(); 
#endif
	//fc_init();

	kprintf("APP: Bettercopter Flight Control\n"); 
	hardware = fc_get_interface();
	fc.SetBoardInterface(&hardware);
	
	gpio_clear(FC_LED_PIN); 
	timestamp_delay_us(1000000L); 
	
	gpio_set(FC_LED_PIN); 
	timestamp_delay_us(500000L); 
	gpio_clear(FC_LED_PIN); 
	timestamp_delay_us(500000L); 
	
	srand(0x1234); 
	
	mPCSerial = hardware.get_pc_link_interface(&hardware); 
	//pc_link.SetSerialInterface(hardware.get_pc_link_interface(&hardware)); 
}

void Application::loop(){
#ifdef CONFIG_SIMULATOR
	sim_loop(); 
#endif
	fc_process_events();
	
	static timestamp_t last_loop = timestamp_now(); 
	uint16_t rc_thr, rc_yaw, rc_pitch, rc_roll, rc_aux0, rc_aux1;
	hardware.read_receiver(&hardware, &rc_thr, &rc_yaw, &rc_pitch, &rc_roll, &rc_aux0, &rc_aux1);
	
	timestamp_t udt = timestamp_ticks_to_us(timestamp_ticks_since(last_loop)); 
	//float dt = udt * 0.000001; 
	last_loop = timestamp_now(); 
	
	fc.update(udt);
	
	PCLinkProcessEvents(); 
	
	static timestamp_t hb_timeout = timestamp_from_now_us(1000000); 
	if(timestamp_expired(hb_timeout)){
		PCLinkSendHeartbeat(); 
		hb_timeout = timestamp_from_now_us(1000000); 
	}
}

void Application::PCLinkSendHeartbeat(){
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
	serial_putn(mPCSerial, buf, len);
}

bool Application::PCLinkReceiveMessage(mavlink_message_t *msg){
	if(!mPCSerial) return false; 
	
	// Example variable, by declaring them static they're persistent
	// and will thus track the system state
	//static int packet_drops = 0;
	//static int mode = MAV_MODE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */

	mavlink_status_t status;
 
	// COMMUNICATION THROUGH EwhileXTERNAL UART PORT (XBee serial)
	
	uint8_t max_count = 16; 
	while(serial_waiting(mPCSerial) && (max_count--))
	{
		uint8_t c = serial_getc(mPCSerial);
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, msg, &status)) {
			return true; 
		}
	}
	// Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
	return false; 
}

void Application::PCLinkProcessEvents(){
	mavlink_message_t msg; 
	if(PCLinkReceiveMessage(&msg)){
		switch(msg.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				// E.g. read GCS heartbeat and go into
				// comm lost mode if timer times out
			}
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
			// EXECUTE ACTION
			break;
		default:
			//Do nothing
			break;
		}
	}
}

/*
/// horribly inefficient with regards to time, but very nice in terms
/// of memory usage. WARNING: Relies on the driver to not do redundant
/// writes. This is how it should be, but keep this in mind! 
void Application::OnPCLinkSetValue(const char *name, const char *value) {
	AppConfig conf; 
	ReadConfig(conf); 
	
	if(pgm_streq(name, PSTR("pid_stab_pitch_p"))){
		sscanf(value, "%f", &conf.pid_stab.pitch.p);
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_i"))){
		sscanf(value, "%f", &conf.pid_stab.pitch.i);
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_d"))){
		sscanf(value, "%f", &conf.pid_stab.pitch.d);
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_max_i"))){
		sscanf(value, "%f", &conf.pid_stab.pitch.max_i);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_p"))){
		sscanf(value, "%f", &conf.pid_stab.yaw.p);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_i"))){
		sscanf(value, "%f", &conf.pid_stab.yaw.i);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_d"))){
		sscanf(value, "%f", &conf.pid_stab.yaw.d);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_max_i"))){
		sscanf(value, "%f", &conf.pid_stab.yaw.max_i);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_p"))){
		sscanf(value, "%f", &conf.pid_stab.roll.p);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_i"))){
		sscanf(value, "%f", &conf.pid_stab.roll.i);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_d"))){
		sscanf(value, "%f", &conf.pid_stab.roll.d);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_max_i"))){
		sscanf(value, "%f", &conf.pid_stab.roll.max_i);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_p"))){
		sscanf(value, "%f", &conf.pid_rate.pitch.p);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_i"))){
		sscanf(value, "%f", &conf.pid_rate.pitch.i);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_d"))){
		sscanf(value, "%f", &conf.pid_rate.pitch.d);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_max_i"))){
		sscanf(value, "%f", &conf.pid_rate.pitch.max_i);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_p"))){
		sscanf(value, "%f", &conf.pid_rate.yaw.p);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_i"))){
		sscanf(value, "%f", &conf.pid_rate.yaw.i);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_d"))){
		sscanf(value, "%f", &conf.pid_rate.yaw.d);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_max_i"))){
		sscanf(value, "%f", &conf.pid_rate.yaw.max_i);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_p"))){
		sscanf(value, "%f", &conf.pid_rate.roll.p);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_i"))){
		sscanf(value, "%f", &conf.pid_rate.roll.i);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_d"))){
		sscanf(value, "%f", &conf.pid_rate.roll.d);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_max_i"))){
		sscanf(value, "%f", &conf.pid_rate.roll.max_i);
	} 
	
	WriteConfig(conf); 
}

int Application::OnPCLinkGetValue(const char *name, char *buffer, uint16_t size) {
	AppConfig conf; 
	//ReadConfig(conf); 
	
	if(pgm_streq(name, PSTR("ping"))){
		return pgm_snprintf(buffer, size, PSTR("pong")); 
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_p"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.pitch.p);
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.pitch.i);
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_d"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.pitch.d);
	} else if(pgm_streq(name, PSTR("pid_stab_pitch_max_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.pitch.max_i);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_p"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.yaw.p);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.yaw.i);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_d"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.yaw.d);
	} else if(pgm_streq(name, PSTR("pid_stab_yaw_max_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.yaw.max_i);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_p"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.roll.p);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.roll.i);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_d"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.roll.d);
	} else if(pgm_streq(name, PSTR("pid_stab_roll_max_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_stab.roll.max_i);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_p"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.pitch.p);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.pitch.i);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_d"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.pitch.d);
	} else if(pgm_streq(name, PSTR("pid_rate_pitch_max_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.pitch.max_i);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_p"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.yaw.p);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.yaw.i);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_d"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.yaw.d);
	} else if(pgm_streq(name, PSTR("pid_rate_yaw_max_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.yaw.max_i);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_p"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.roll.p);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.roll.i);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_d"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.roll.d);
	} else if(pgm_streq(name, PSTR("pid_rate_roll_max_i"))){
		return pgm_snprintf(buffer, size, PSTR("%f"), (double)conf.pid_rate.roll.max_i);
	} 
	return 0; 
}
*/
void Application::WriteConfig(const AppConfig &conf){
	struct fc_quad_interface *mBoard = &hardware; 
	mBoard->write_config(mBoard, (uint8_t*)&conf, sizeof(AppConfig)); 
}
	
void Application::ReadConfig(AppConfig &conf){
	struct fc_quad_interface *mBoard = &hardware; 
	mBoard->read_config(mBoard, (uint8_t*)&conf, sizeof(AppConfig)); 
}

extern "C" void app_init(void){
	app.init();
}

extern "C" void app_process_events(void){
	app.loop();
}
