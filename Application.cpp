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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "Application.hpp"
#include "FlightController.hpp"
#include "PCLink.hpp"
#include "ArmSwitch.hpp"
#include "LedIndicator.hpp"
#include "types.hpp"

#include "mavlink.h"

#define TXRX_NUM_CHANNELS 10
#define DEVICE_TWI_ADDRESS 0x88

void sim_init(); 
void sim_loop(); 

#define CONFIG_VERSION 0x55

/// 2600kv pids
const AppConfig default_config PROGMEM = {
	CONFIG_VERSION, 
	// pid_stab
	{
		{0, 0, 0, 30}, // yaw
		{0, 0.0, 0.0, 30}, // pitch
		{0, 0.0, 0.0, 30} // roll
	}, 
	// pid_rate
	{
		{1, 0.0, 2.0, 500}, // yaw
		{70.0, 0, 0, 100}, // pitch
		{70.0, 0.1, 15.0, 100} // roll
	}
};  

/// 1400kv pids
/*
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
*/

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
	LedIndicator 	mLED; 
	serial_dev_t 	mPCSerial; 
	copter_state_t mState; 
};

static Application app;

void Application::init(){
#ifdef CONFIG_SIMULATOR
	sim_init(); 
#endif
	//fc_init();
	
	
	kdebug("BetterCopter V1.0\n");
	 
	mBoard = fc_interface();
	if(!mBoard) {
		//kprintf("No board!\n"); 
		while(1); 
	}
	
	mPCSerial = fc_get_pc_link_interface(mBoard); 

	mPCLink.SetSerialInterface(mPCSerial); 
	fc_led_off(); 
	
	for(int c = 0; c < 3; c++){
		fc_led_on(); 
		timestamp_delay_us(200000L); 
		fc_led_off(); 
		timestamp_delay_us(200000L); 
	}
	
	//srand(0x1234); 
	
	//fc_calibrate_escs(); 
	
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
	//glm::vec3 acc, gyr, mag; 
	
	mLED.Update(); 
	
	fc_process_events();
	
	static timestamp_t last_loop = timestamp_now(); 
	timestamp_t udt = timestamp_ticks_to_us(timestamp_ticks_since(last_loop)); 
	last_loop = timestamp_now(); 
	
	float dt = udt * 0.000001; 
	
	struct fc_data sensors; 
	fc_read_sensors(mBoard, &sensors); 
	fc_read_receiver(mBoard, &rc.throttle, &rc.yaw, &rc.pitch, &rc.roll, &rc.aux0, &rc.aux1);
	
	float altitude = sensors.sonar_altitude; 
	
	if(!mArmSwitch.IsArmed() && mArmSwitch.TryArm(rc)){
		fc.Reset(); 
		mLED.On(); 
		mState = COPTER_STATE_ACTIVE; 
	} else if(mArmSwitch.IsArmed() && mArmSwitch.TryDisarm(rc)){
		mLED.Off(); 
		mState = COPTER_STATE_STANDBY; 
	}
	
	/*
	static float step = 0; 
	static timestamp_t test_timeout = timestamp_now();
	static timestamp_t next_step_time = -1; 
	static timestamp_t switch_time = -1; 
	static const float step_increase = 0.06; 
	static const int32_t test_length = 10000000L; 
	static const int32_t timeout = 100000L; 
	static float sign = 1; 
	static int test_throttle = MINCOMMAND; 
	static int roll_offset = 250; 
	if(test_timeout != -1 && timestamp_expired(test_timeout)){
		float change = sin(step) * 350; 
		if(step > M_PI) step = 0; 
		step += step_increase * sign; 
		if(step > M_PI / 2) {
			test_timeout = -1; 
			next_step_time = timestamp_from_now_us(test_length); 
			switch_time = timestamp_from_now_us(test_length / 3); 
			roll_offset = 250; 
		} else if(step < 0){
			step = 0; 
			sign = -sign; 
		} else {
			next_step_time = -1; 
			test_timeout = timestamp_from_now_us(timeout); 
			test_throttle = 1050 + change; 
			if(test_throttle < 1050) test_throttle = 1000; 
		}
	} 
	if(next_step_time != -1 && timestamp_expired(next_step_time)){
		sign = -sign; 
		test_timeout = timestamp_from_now_us(timeout); 
		next_step_time = -1; 
		switch_time = -1; 
		roll_offset = 0; 
	}
	if(switch_time != -1 && timestamp_expired(switch_time)){
		switch_time = timestamp_from_now_us(test_length / 3); 
		roll_offset -= 250; 
	}
	rc.roll = 1500;// + roll_offset; 
	rc.throttle = test_throttle; 
	rc.pitch = rc.yaw = 1500; 
	mArmSwitch.On(); 
	mState = COPTER_STATE_ACTIVE; 
	*/
	ThrottleValues throttle = 
		fc.ComputeThrottle(dt, 
			rc, 
			glm::vec3(sensors.acc_g.x, sensors.acc_g.y, sensors.acc_g.z), 
			glm::vec3(sensors.gyr_deg.x, sensors.gyr_deg.y, sensors.gyr_deg.z),
			glm::vec3(sensors.mag.x, sensors.mag.y, sensors.mag.z),
			altitude); 
	
	if(rc.throttle > 1050 && mArmSwitch.IsArmed()){
		//MINCOMMAND, MINCOMMAND); // MINCOMMAND, MINCOMMAND/*
		fc_write_motors(mBoard, 
			//test_throttle, MINCOMMAND, MINCOMMAND, MINCOMMAND); //
			throttle[0], throttle[1], throttle[2], throttle[3]);
	} else {
		fc_write_motors(mBoard, 
			MINCOMMAND, MINCOMMAND, MINCOMMAND, MINCOMMAND);
	}
	
	PCLinkProcessEvents(); 
	
	static timestamp_t data_timeout = timestamp_from_now_us(100000); 
	if(timestamp_expired(data_timeout)){	
		// batval is in percentage of board supply voltage, so need to scale
		float vbat = sensors.vbat * 15; 
		if(vbat < 10.8) {
			// battery is too low! 
			mLED.FastBlink(); 
		} else if(vbat > 5 && vbat < 9){
			//Disarm(); 
		} else {
			if(mArmSwitch.IsArmed()) mLED.On(); 
			else mLED.Off(); 
		}
		
		mPCLink.SendPowerStatus(vbat, 4.9); 
		
		mPCLink.SendRCChannels(loop_nr, rc.throttle, rc.yaw, rc.pitch, rc.roll, 0, 0); 
		
		mPCLink.SendRawIMU(loop_nr, 
			glm::vec3(sensors.raw_acc.x, sensors.raw_acc.y, sensors.raw_acc.z), 
			glm::vec3(sensors.raw_gyr.x, sensors.raw_gyr.y, sensors.raw_gyr.z),
			glm::vec3(sensors.raw_mag.x, sensors.raw_mag.y, sensors.raw_mag.z)
		); 
		
		mPCLink.SendScaledIMU(loop_nr, 
			glm::vec3(sensors.acc_g.x, sensors.acc_g.y, sensors.acc_g.z), 
			glm::vec3(sensors.gyr_deg.x, sensors.gyr_deg.y, sensors.gyr_deg.z),
			glm::vec3(sensors.mag.x, sensors.mag.y, sensors.mag.z)
		); 
		//static float yaw = 0; 
		
		glm::vec3 nacc = glm::normalize(glm::vec3(sensors.acc_g.x, sensors.acc_g.y, sensors.acc_g.z)); 
		float pitch = glm::radians(fc.GetPitch()); //::atan2(nacc.y , nacc.z ); 
		float roll = glm::radians(fc.GetRoll()); //::atan2(nacc.x , nacc.z ); 
		float yaw = glm::radians(fc.GetYaw()); //::atan2(sensors.mag.y, sensors.mag.x); 
		
		mPCLink.SendMotorOutput(loop_nr, throttle[0], throttle[1], 
			throttle[2], throttle[3]); 
		
		mPCLink.SendAttitude(loop_nr, 
			yaw, pitch, roll, 
			glm::radians(sensors.gyr_deg.z), 
			glm::radians(sensors.gyr_deg.x), 
			glm::radians(sensors.gyr_deg.y)); 
		
		uint16_t avg_thr = (
			constrain(
				(throttle[0] + throttle[1] + throttle[2] + throttle[3]) >> 2, 
				1000, 2000
			) - 1000) / 10; 
		mPCLink.SendHud(dt, 0, yaw, avg_thr, altitude, pitch); 
		
		data_timeout = timestamp_from_now_us(100000); 
	}
	
	static timestamp_t hb_timeout = timestamp_from_now_us(5000000L); 
	if(timestamp_expired(hb_timeout)){
		//mPCLink.SendParamValueFloat("MEM_FREE", StackCount(), 1, 0); 
		mPCLink.SendHeartbeat(mState); 
		hb_timeout = timestamp_from_now_us(1000000); 
	}
	
	//printf("Stack: %u\n", StackCount()); 
}

void Application::PCLinkProcessEvents(){
	mavlink_message_t msg; 
	AppConfig conf; 
	struct param {
		const char *name; 
		float *value; 
	} params [] = {
		{PSTR("RATE.Y_P"), &conf.pid_rate.yaw.p},
		{PSTR("RATE.Y_I"), &conf.pid_rate.yaw.i},
		{PSTR("RATE.Y_D"), &conf.pid_rate.yaw.d},
		{PSTR("RATE.Y_MAX_I"), &conf.pid_rate.yaw.max_i},
		{PSTR("RATE.P_P"), &conf.pid_rate.pitch.p},
		{PSTR("RATE.P_I"), &conf.pid_rate.pitch.i},
		{PSTR("RATE.P_D"), &conf.pid_rate.pitch.d},
		{PSTR("RATE.P_MAX_I"), &conf.pid_rate.pitch.max_i},
		{PSTR("RATE.R_P"), &conf.pid_rate.roll.p},
		{PSTR("RATE.R_I"), &conf.pid_rate.roll.i},
		{PSTR("RATE.R_D"), &conf.pid_rate.roll.d},
		{PSTR("RATE.R_MAX_I"), &conf.pid_rate.roll.max_i},
		{PSTR("STAB.Y_P"), &conf.pid_stab.yaw.p},
		{PSTR("STAB.Y_I"), &conf.pid_stab.yaw.i},
		{PSTR("STAB.Y_D"), &conf.pid_stab.yaw.d}, 
		{PSTR("STAB.Y_MAX_I"), &conf.pid_stab.yaw.max_i},
		{PSTR("STAB.P_P"), &conf.pid_stab.pitch.p},
		{PSTR("STAB.P_I"), &conf.pid_stab.pitch.i},
		{PSTR("STAB.P_D"), &conf.pid_stab.pitch.d},
		{PSTR("STAB.P_MAX_I"), &conf.pid_stab.pitch.max_i},
		{PSTR("STAB.R_P"), &conf.pid_stab.roll.p},
		{PSTR("STAB.R_I"), &conf.pid_stab.roll.i},
		{PSTR("STAB.R_D"), &conf.pid_stab.roll.d},
		{PSTR("STAB.R_MAX_I"), &conf.pid_stab.roll.max_i},
		//{PSTR("MEM_FREE"), (float)StackCount()}
	};
	
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
			ReadConfig(&conf); 
			
			int count = sizeof(params) / sizeof(params[0]); 
			
			if(msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST){
				for(int c = 0; c < count; c++){
					char name[16]; 
					strcpy_PF(name, (uint_farptr_t)params[c].name); 
					mPCLink.SendParamValueFloat(
						name, *params[c].value, 
						count, c); 
				}
			} else {
				mavlink_param_request_read_t param; 
				mavlink_msg_param_request_read_decode(&msg, &param); 
				int idx = param.param_index - 1; 
				if(idx >= 0 && idx < count){
					char name[16]; 
					strcpy_PF(name, (uint_farptr_t)params[idx].name); 
					mPCLink.SendParamValueFloat(
							name, *params[idx].value, 
							count, idx); 
				} 
			}
			break; 
		}
		case MAVLINK_MSG_ID_PARAM_SET: {
			mavlink_param_set_t param; 
			mavlink_msg_param_set_decode(&msg, &param); 
			ReadConfig(&conf); 
			
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

int main(){
	fc_init(); 
	app.init(); 
	while(1){
		app.loop(); 
	}
	return 0; 
}
