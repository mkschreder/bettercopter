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
#include "PCLink.hpp"
#include "ArmSwitch.hpp"
#include "types.hpp"

#include "mavlink.h"

#define TXRX_NUM_CHANNELS 10
#define DEVICE_TWI_ADDRESS 0x88


void sim_init(); 
void sim_loop(); 

#define CONFIG_VERSION 0x55

const AppConfig default_config PROGMEM = {
	CONFIG_VERSION, 
	// pid_stab
	{
		{0.2, 0, 0, 30}, // yaw
		{5.0, 0.01, 1.0, 30}, // pitch
		{5.0, 0.01, 1.0, 30} // roll
	}, 
	// pid_rate
	{
		{30, 0.0, 2.0, 500}, // yaw
		{0.8, 0.01, 0.04, 500}, // pitch
		{0.8, 0.01, 0.04, 500} // roll
	}
};  
 
class Application {
public:
	Application():mState(COPTER_STATE_BOOT) {}
	void init();
	void loop();
	
public: 
	void PCLinkProcessEvents(); 
	
	void WriteConfig(const AppConfig &conf); 
	void ReadConfig(AppConfig *conf); 
	void ApplyConfig(const AppConfig &conf); 
private:
	fc_board_t 		mBoard; 
	uint8_t armed, arm_progress; 
	timestamp_t arm_timeout; 
	FlightController fc; 
	PCLink				mPCLink; 
	ArmSwitch 		mArmSwitch; 
	serial_dev_t 	mPCSerial; 
	copter_state_t mState; 
};

static Application app;

void Application::init(){
#ifdef CONFIG_SIMULATOR
	sim_init(); 
#endif
	//fc_init();

	kprintf("BetterCopter V1.0\n"); 
	mBoard = fc_get_interface();
	if(!mBoard) {
		//kprintf("No board!\n"); 
		while(1); 
	}
	gpio_clear(FC_LED_PIN); 
	timestamp_delay_us(1000000L); 
	
	gpio_set(FC_LED_PIN); 
	timestamp_delay_us(500000L); 
	gpio_clear(FC_LED_PIN); 
	timestamp_delay_us(500000L); 
	
	srand(0x1234); 
	
	mPCSerial = fc_get_pc_link_interface(mBoard); 
	mPCLink.SetSerialInterface(mPCSerial); 
	
	AppConfig conf; 
	ReadConfig(&conf); 
	ApplyConfig(conf); 
	
	mState = COPTER_STATE_STANDBY; 
}

void Application::loop(){
#ifdef CONFIG_SIMULATOR
	sim_loop(); 
#endif
	static uint32_t loop_nr = 0; loop_nr++; 
	RCValues rc; 
	glm::vec3 acc, gyr, mag; 
	
	fc_process_events();
	
	static timestamp_t last_loop = timestamp_now(); 
	timestamp_t udt = timestamp_ticks_to_us(timestamp_ticks_since(last_loop)); 
	last_loop = timestamp_now(); 
	
	float dt = udt * 0.000001; 
	
	fc_read_receiver(mBoard, &rc.throttle, &rc.yaw, &rc.pitch, &rc.roll, &rc.aux0, &rc.aux1);
	fc_read_accelerometer(mBoard, &acc.x, &acc.y, &acc.z);
	fc_read_gyroscope(mBoard, &gyr.x, &gyr.y, &gyr.z);
	float altitude = 	fc_read_altitude(mBoard); 
	//long pressure = 	fc_read_pressure(mBoard); 
	//float temp = 			fc_read_temperature(mBoard); 
	
	if(!mArmSwitch.IsArmed() && mArmSwitch.TryArm(rc)){
		fc.Reset(); 
		gpio_set(FC_LED_PIN); 
		mState = COPTER_STATE_ACTIVE; 
	} else if(mArmSwitch.IsArmed() && mArmSwitch.TryDisarm(rc)){
		gpio_clear(FC_LED_PIN); 
		mState = COPTER_STATE_STANDBY; 
	}
	
	ThrottleValues throttle = ThrottleValues(MINCOMMAND); 
	
	if(rc.throttle > 1050 && mArmSwitch.IsArmed()){
		throttle = fc.ComputeThrottle(dt, 
			rc, acc, gyr, mag, altitude); 
	}
	
	fc_write_motors(mBoard, 
			throttle[0], throttle[1], throttle[2], throttle[3]);
	
	PCLinkProcessEvents(); 
	
	static timestamp_t data_timeout = timestamp_from_now_us(100000); 
	if(timestamp_expired(data_timeout)){	
		mPCLink.SendRawIMU(loop_nr, acc, gyr, mag); 
		
		float yaw, pitch, roll, rate_yaw, rate_pitch, rate_roll; 
		fc.ComputeAngles(dt, acc, gyr,
			&yaw, &pitch, &roll, 
			&rate_yaw, &rate_pitch, &rate_roll); 
		
		mPCLink.SendAttitude(loop_nr, 
			yaw, pitch, roll, 
			rate_yaw, rate_pitch, rate_roll); 
		uint16_t avg_thr = (
			constrain(
				(throttle[0] + throttle[1] + throttle[2] + throttle[3]) >> 2, 
				1000, 2000
			) - 1000) / 10; 
		mPCLink.SendHud(0, 0, yaw, avg_thr, altitude, pitch); 
		
		data_timeout = timestamp_from_now_us(100000); 
	}
	
	static timestamp_t hb_timeout = timestamp_from_now_us(1000000); 
	if(timestamp_expired(hb_timeout)){
		//mPCLink.SendParamValueFloat("MEM_FREE", StackCount(), 1, 0); 
		mPCLink.SendHeartbeat(mState); 
		hb_timeout = timestamp_from_now_us(1000000); 
	}
}

void Application::PCLinkProcessEvents(){
	mavlink_message_t msg; 
	if(mPCLink.ReceiveMessage(&msg)){
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
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: 
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			AppConfig conf; 
			ReadConfig(&conf); 
			struct param {
				const char *name; 
				float value; 
			} params [] = {
				{PSTR("RATE.YAW_P"), conf.pid_rate.yaw.p},
				{PSTR("RATE.YAW_I"), conf.pid_rate.yaw.i},
				{PSTR("RATE.YAW_D"), conf.pid_rate.yaw.d},
				{PSTR("RATE.YAW_MAX_I"), conf.pid_rate.yaw.max_i},
				{PSTR("RATE.PITCH_P"), conf.pid_rate.pitch.p},
				{PSTR("RATE.PITCH_I"), conf.pid_rate.pitch.i},
				{PSTR("RATE.PITCH_D"), conf.pid_rate.pitch.d},
				{PSTR("RATE.PITCH_MAX_I"), conf.pid_rate.pitch.max_i},
				{PSTR("RATE.ROLL_P"), conf.pid_rate.roll.p},
				{PSTR("RATE.ROLL_I"), conf.pid_rate.roll.i},
				{PSTR("RATE.ROLL_D"), conf.pid_rate.roll.d},
				{PSTR("RATE.ROLL_MAX_I"), conf.pid_rate.roll.max_i},
				{PSTR("STAB.YAW_P"), conf.pid_stab.yaw.p},
				{PSTR("STAB.YAW_I"), conf.pid_stab.yaw.i},
				{PSTR("STAB.YAW_D"), conf.pid_stab.yaw.d}, 
				{PSTR("STAB.YAW_MAX_I"), conf.pid_stab.yaw.max_i},
				{PSTR("STAB.PITCH_P"), conf.pid_stab.pitch.p},
				{PSTR("STAB.PITCH_I"), conf.pid_stab.pitch.i},
				{PSTR("STAB.PITCH_D"), conf.pid_stab.pitch.d},
				{PSTR("STAB.PITCH_MAX_I"), conf.pid_stab.pitch.max_i},
				{PSTR("STAB.ROLL_P"), conf.pid_stab.roll.p},
				{PSTR("STAB.ROLL_I"), conf.pid_stab.roll.i},
				{PSTR("STAB.ROLL_D"), conf.pid_stab.roll.d},
				{PSTR("STAB.ROLL_MAX_I"), conf.pid_stab.roll.max_i},
				{PSTR("MEM_FREE"), (float)StackCount()}
			};
			int count = sizeof(params) / sizeof(params[0]); 
			/*for(int c = 0; c < count; c++){
				char name[16]; 
				strcpy_PF(name, (uint_farptr_t)params[c].name); 
				mPCLink.SendParamValueFloat(
					name, params[c].value, 
					count, c); 
			}*/
				
			if(msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST){
				for(int c = 0; c < count; c++){
					char name[16]; 
					strcpy_PF(name, (uint_farptr_t)params[c].name); 
					mPCLink.SendParamValueFloat(
						name, params[c].value, 
						count, c); 
				}
			} else {
				mavlink_param_request_read_t param; 
				mavlink_msg_param_request_read_decode(&msg, &param); 
				if(param.param_index > 0 && param.param_index <= count){
					char name[16]; 
					strcpy_PF(name, (uint_farptr_t)params[param.param_index - 1].name); 
					mPCLink.SendParamValueFloat(
							name, params[param.param_index - 1].value, 
							count, param.param_index - 1); 
				} 
			}
			break; 
		}
		case MAVLINK_MSG_ID_PARAM_SET: {
			mavlink_param_set_t param; 
			mavlink_msg_param_set_decode(&msg, &param); 
			AppConfig conf; 
			ReadConfig(&conf); 
			float dummy; 
			struct param {
				const char *name; 
				float *value; 
			} params [] = {
				{PSTR("RATE.YAW_P"), &conf.pid_rate.yaw.p},
				{PSTR("RATE.YAW_I"), &conf.pid_rate.yaw.i},
				{PSTR("RATE.YAW_D"), &conf.pid_rate.yaw.d},
				{PSTR("RATE.YAW_MAX_I"), &conf.pid_rate.yaw.max_i},
				{PSTR("RATE.PITCH_P"), &conf.pid_rate.pitch.p},
				{PSTR("RATE.PITCH_I"), &conf.pid_rate.pitch.i},
				{PSTR("RATE.PITCH_D"), &conf.pid_rate.pitch.d},
				{PSTR("RATE.PITCH_MAX_I"), &conf.pid_rate.pitch.max_i},
				{PSTR("RATE.ROLL_P"), &conf.pid_rate.roll.p},
				{PSTR("RATE.ROLL_I"), &conf.pid_rate.roll.i},
				{PSTR("RATE.ROLL_D"), &conf.pid_rate.roll.d},
				{PSTR("RATE.ROLL_MAX_I"), &conf.pid_rate.roll.max_i},
				{PSTR("STAB.YAW_P"), &conf.pid_stab.yaw.p},
				{PSTR("STAB.YAW_I"), &conf.pid_stab.yaw.i},
				{PSTR("STAB.YAW_D"), &conf.pid_stab.yaw.d}, 
				{PSTR("STAB.YAW_MAX_I"), &conf.pid_stab.yaw.max_i},
				{PSTR("STAB.PITCH_P"), &conf.pid_stab.pitch.p},
				{PSTR("STAB.PITCH_I"), &conf.pid_stab.pitch.i},
				{PSTR("STAB.PITCH_D"), &conf.pid_stab.pitch.d},
				{PSTR("STAB.PITCH_MAX_I"), &conf.pid_stab.pitch.max_i},
				{PSTR("STAB.ROLL_P"), &conf.pid_stab.roll.p},
				{PSTR("STAB.ROLL_I"), &conf.pid_stab.roll.i},
				{PSTR("STAB.ROLL_D"), &conf.pid_stab.roll.d},
				{PSTR("STAB.ROLL_MAX_I"), &conf.pid_stab.roll.max_i},
				{PSTR("MEM_FREE"), &dummy}
			};
			int count = sizeof(params) / sizeof(params[0]); 
			for(int c = 0; c < count; c++){
				if(strcmp_PF(param.param_id, (uint_farptr_t)params[c].name) == 0){
					*(params[c].value) = param.param_value; 
					mPCLink.SendParamValueFloat(param.param_id, param.param_value, 1, 0); 
					WriteConfig(conf); 
					ApplyConfig(conf); 
					break; 
				} 
			}
			break; 
		}
		default:
			//Do nothing
			break;
		}
	}
}

void Application::WriteConfig(const AppConfig &conf){
	fc_write_config(mBoard, (uint8_t*)&conf, sizeof(AppConfig)); 
}
	
void Application::ReadConfig(AppConfig *conf){ 
	fc_read_config(mBoard, (uint8_t*)conf, sizeof(AppConfig)); 
	// make sure to reset the config if data in eeprom is garbage
	if(conf->version != CONFIG_VERSION){
		memcpy_PF(conf, (uint_farptr_t)&default_config, sizeof(AppConfig)); 
		WriteConfig(*conf); 
	} 
}

void Application::ApplyConfig(const AppConfig &conf){
	fc.SetPIDValues(
		conf.pid_stab.yaw,
		conf.pid_stab.pitch, 
		conf.pid_stab.roll, 
		conf.pid_rate.yaw, 
		conf.pid_rate.pitch, 
		conf.pid_rate.roll); 
}

extern "C" void app_init(void){
	app.init();
}

extern "C" void app_process_events(void){
	app.loop();
}
