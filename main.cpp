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

#ifdef CONFIG_SIMULATOR
#include "simulator/sim_kernel.h"
#endif

/*
#include <net/tcpip.h>
#include <net/rfnet.h>*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "FlightController.hpp"

#define TXRX_NUM_CHANNELS 10
#define DEVICE_TWI_ADDRESS 0x88

//static struct uart uart;

static uint8_t armed = 0, arm_progress = 0; 
static timeout_t arm_timeout = 0; 
//static IMU imu; 
static FlightController fc; 

void sim_init(); 
void sim_loop(); 

static glm::vec3 ofs; 
	
extern "C" void Sleep( volatile uint32_t dwMs ) ;

extern "C" void app_init(void){
	
#ifdef CONFIG_SIMULATOR
	sim_init(); 
#endif

	set_pin(LED_PIN, 0); 
	time_delay(1000000); 
	
	set_pin(LED_PIN, 1); 
	time_delay(500000); 
	set_pin(LED_PIN, 0); 
	time_delay(500000); 
	set_pin(LED_PIN, 1); 
	
	srand(0x1234); 
	/*
	kdebug("WAITING...\n"); 
	timeout_t tim = timeout_from_now(1000000); 
	while(!timeout_expired(tim)){
		kdebug("US: %d %d\n", time_get_clock(), time_clock_to_us(time_clock_since(tim))); 
	}
	kdebug("DONE!\n"); */
	//fc.setSensorProvider(&imu); 
	
	set_pin(PWM_FRONT, MINCOMMAND); 
	set_pin(PWM_BACK, MINCOMMAND); 
	set_pin(PWM_LEFT, MINCOMMAND); 
	set_pin(PWM_RIGHT, MINCOMMAND); 
	
	for(int c = 0; c < 50; c++){
		float ax, ay, az; 
		get_accelerometer(&ax, &ay, &az);
		ofs += glm::vec3((float)ax, (float)ay, (float)az); 
		kprintf("ofs: %-4d %-4d %-4d\n", 
		(int16_t)(ax * 100), (int16_t)(ay* 100),(int16_t)( az*100)); 
		
		time_delay(10000); 
	}
	ofs = ofs * 0.02f; ofs.z = 0; 
	kprintf("AOFS: %-4d %-4d %-4d\n", 
		(int16_t)(ofs.x), (int16_t)(ofs.y), (int16_t)(ofs.z)); 
		
	//imu.setAccelOffset(ofs); 
	
	//uart_printf(PSTR("READY!\n")); 
}

extern "C" void app_loop(void){
#ifdef CONFIG_SIMULATOR
	sim_loop(); 
#endif
	//PORTC |= _BV(0); 
	//DDRC |= _BV(0); 
	
	glm::vec3 acc, gyr, mag(1, 0, 0); 
	int16_t A = 0, P = 0, T = 0; 
	
	//int16_t throttle, yaw, pitch, roll; 
	static float rc_throttle, rc_yaw, rc_pitch, rc_roll, rc_mode; 
	
	static timeout_t last_loop = time_get_clock(); 
	
	timeout_t udt = time_clock_to_us(time_clock_since(last_loop)); 
	float dt = udt * 0.000001; 
	last_loop = time_get_clock(); 
	
	get_accelerometer(&acc.x, &acc.y, &acc.z); 
	//acc.x -= ofs.x; acc.y -= ofs.y; 
	float tmp = acc.z; acc.z = acc.y; acc.y = tmp; 
	
	get_gyroscope(&gyr.x, &gyr.z, &gyr.y);
	//get_magnetometer(&mx, &my, &mz);
	//get_altitude(&A); 
	//get_pressure(&P); 
	//get_temperature(&T); 
	
	/*rc_throttle = 0.85f * rc_throttle + 0.15f * get_pin(RC_THROTTLE); 
	rc_yaw = 0.85f * rc_yaw + 0.15f * get_pin(RC_YAW); 
	rc_pitch = 0.85f * rc_pitch + 0.15f * get_pin(RC_PITCH); 
	rc_roll = 0.85f * rc_roll + 0.15f * get_pin(RC_ROLL); */
	rc_throttle = get_pin(RC_THROTTLE); 
	rc_yaw = get_pin(RC_YAW); 
	rc_pitch = get_pin(RC_PITCH); 
	rc_roll = get_pin(RC_ROLL); 
	rc_mode = get_pin(RC_MODE); 
	
	// prevent small changes when stick is not touched
	if(abs(rc_pitch - 1500) < 20) rc_pitch = 1500; 
	if(abs(rc_roll - 1500) < 20) rc_roll = 1500; 
	if(abs(rc_yaw - 1500) < 20) rc_yaw = 1500; 
	
	fc.updateSensors(acc, gyr, mag, A, P, T); 
	fc.update(rc_throttle, rc_yaw, rc_pitch, rc_roll, rc_mode, udt);
	
	glm::i16vec4 thr = fc.getMotorThrust(); 
	
	if(armed && rc_throttle > 1050){ // prevent spin when arming!
		set_pin(PWM_FRONT, thr[0]); 
		set_pin(PWM_BACK, thr[1]); 
		set_pin(PWM_RIGHT, thr[2]); 
		set_pin(PWM_LEFT, thr[3]); 
	} else {
		set_pin(PWM_FRONT, MINCOMMAND); 
		set_pin(PWM_BACK, MINCOMMAND); 
		set_pin(PWM_RIGHT, MINCOMMAND); 
		set_pin(PWM_LEFT, MINCOMMAND); 
	}
	
	// arming sequence 
	if(!armed && rc_throttle < 1050 && rc_roll > 1700){
		if(!arm_progress) {
			arm_timeout = timeout_from_now(1000000); 
			arm_progress = 1; 
		} else if(timeout_expired(arm_timeout)){
			kdebug("ARMED!\n"); 
			fc.reset(); 
			set_pin(LED_PIN, 1); 
			armed = 1; 
			arm_progress = 0; 
		}
	} else if(armed && rc_throttle < 1050 && rc_roll < 1100){
		if(!arm_progress) {
			arm_timeout = timeout_from_now(1000000); 
			arm_progress = 1; 
		} else if(timeout_expired(arm_timeout)){
			set_pin(LED_PIN, 0); 
			armed = 0; 
			arm_progress = 0; 
		}
	} else if(timeout_expired(arm_timeout)){
		arm_progress = 0; 
	}
	//PORTC &= ~_BV(0); 
	
	//get_magnetometer(&mx, &my, &mz);
	kdebug("DT: %lu, ", udt); 
	
	kdebug("A: [%d, %d, %d]\n", 
		ax, 
		ay, 
		az);
	
	kdebug("Gx100: [%d, %d, %d]\n", 
		(int16_t)(gx * 100.0f), 
		(int16_t)(gy * 100.0f), 
		(int16_t)(gz * 100.0f)
	);
	kdebug("M: [%d, %d, %d]\n", 
		mx, 
		my, 
		mz);
	kdebug("RC: [%-4d, %-4d, %-4d, %-4d] ", 
		(uint16_t)rc_throttle, 
		(uint16_t)rc_yaw, 
		(uint16_t)rc_pitch, 
		(uint16_t)rc_roll, 
		(uint16_t)rc_mode); 
	kdebug("THR: [%-4d, %-4d, %-4d, %-4d]\n", 
		(armed)?(uint16_t)thr[0]:MINCOMMAND, 
		(armed)?(uint16_t)thr[1]:MINCOMMAND, 
		(armed)?(uint16_t)thr[2]:MINCOMMAND, 
		(armed)?(uint16_t)thr[3]:MINCOMMAND);
}

DECLARE_MAIN_MODULE() {
	.init = app_init,
	.loop = app_loop
}; 


