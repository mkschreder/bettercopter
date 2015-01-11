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
	void PCLinkProcessEvents(); 
	
	void WriteConfig(const AppConfig &conf); 
	void ReadConfig(AppConfig &conf); 
private:
	fc_board_t 		mBoard; 
	uint8_t armed, arm_progress; 
	timestamp_t arm_timeout; 
	FlightController fc; 
	PCLink				mPCLink; 
	ArmSwitch 		mArmSwitch; 
	serial_dev_t 	mPCSerial; 
};

static Application app;

void Application::init(){
#ifdef CONFIG_SIMULATOR
	sim_init(); 
#endif
	//fc_init();

	kprintf("APP: Bettercopter Flight Control\n"); 
	mBoard = fc_get_interface();
	
	gpio_clear(FC_LED_PIN); 
	timestamp_delay_us(1000000L); 
	
	gpio_set(FC_LED_PIN); 
	timestamp_delay_us(500000L); 
	gpio_clear(FC_LED_PIN); 
	timestamp_delay_us(500000L); 
	
	srand(0x1234); 
	
	mPCSerial = fc_get_pc_link_interface(mBoard); 
	//pc_link.SetSerialInterface(hardware.get_pc_link_interface(&hardware)); 
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
	} else if(mArmSwitch.IsArmed() && mArmSwitch.TryDisarm(rc)){
		gpio_clear(FC_LED_PIN); 
	}
	
	ThrottleValues throttle = ThrottleValues(MINCOMMAND); 
	
	if(mArmSwitch.IsArmed()){
		throttle = fc.ComputeThrottle(dt, 
			rc, acc, gyr, mag, altitude); 
	}
	
	fc_write_motors(mBoard, 
			throttle[0], throttle[1], throttle[2], throttle[3]);
	
	PCLinkProcessEvents(); 
	
	mPCLink.SendRawIMU(loop_nr, acc, gyr, mag); 
	/*mPCLink.SendAttitude(loop_nr, 
		roll, pitch, yaw, 
		rate_roll, rate_pitch, rate_yaw); 
	*/	
	static timestamp_t hb_timeout = timestamp_from_now_us(1000000); 
	if(timestamp_expired(hb_timeout)){
		mPCLink.SendHeartbeat(); 
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
	fc_write_config(mBoard, (uint8_t*)&conf, sizeof(AppConfig)); 
}
	
void Application::ReadConfig(AppConfig &conf){ 
	fc_read_config(mBoard, (uint8_t*)&conf, sizeof(AppConfig)); 
}

extern "C" void app_init(void){
	app.init();
}

extern "C" void app_process_events(void){
	app.loop();
}
