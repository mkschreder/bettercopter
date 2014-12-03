/*********************************************

For more projects, visit https://github.com/mkschreder/

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**********************************************/
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

extern "C" void app_init(void){
#ifdef CONFIG_SIMULATOR
	sim_init(); 
#endif

	time_delay(1000000); 
	
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
	
	glm::vec3 ofs; 
	for(int c = 0; c < 10; c++){
		int16_t ax, ay, az; 
		get_accelerometer(&ax, &ay, &az);
		ofs += glm::vec3((float)ax, (float)ay, 0); 
		//_delay_ms(5); 
	}
	ofs = ofs * 0.1f; 
	//imu.setAccelOffset(ofs); 
	
	//uart_printf(PSTR("READY!\n")); 
}

extern "C" void app_loop(void){
#ifdef CONFIG_SIMULATOR
	sim_loop(); 
#endif
	//PORTC |= _BV(0); 
	//DDRC |= _BV(0); 
	
	int16_t ax, ay, az; 
	float gx, gy, gz;  
	int16_t mx = 1, my = 0, mz = 0; 
	int16_t A = 0, P = 0, T = 0; 
	//int16_t throttle, yaw, pitch, roll; 
	uint16_t rc_throttle, rc_yaw, rc_pitch, rc_roll, rc_mode; 
	
	static timeout_t last_loop = time_get_clock(); 
	
	timeout_t udt = time_clock_to_us(time_clock_since(last_loop)); 
	float dt = udt * 0.000001; 
	last_loop = time_get_clock(); 
	
	get_accelerometer(&ax, &ay, &az); 
	int16_t tmp = az; az = ay; ay = tmp; 
	get_gyroscope(&gx, &gz, &gy);
	//get_magnetometer(&mx, &my, &mz);
	//get_altitude(&A); 
	//get_pressure(&P); 
	//get_temperature(&T); 
	
	rc_throttle = get_pin(RC_THROTTLE); 
	rc_yaw = get_pin(RC_YAW); 
	rc_pitch = get_pin(RC_PITCH); 
	rc_roll = get_pin(RC_ROLL); 
	rc_mode = get_pin(RC_MODE); 
	//uint8_t rc_active = get_rc_commands(&throttle, &yaw, &pitch, &roll); 
	
	//imu.update(ax, ay, az, gx, gy, gz, mx, my, mz, A, P, T, dt); 
	fc.updateSensors(
		glm::vec3(ax, ay, az), 
		glm::vec3(gx, gy, gz), 
		glm::vec3(mx, my, mz), A, P, T); 
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
	
	// check if being armed
	if(!armed && rc_throttle < 1050 && rc_roll > 1800){
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
	} else if(armed && rc_throttle < 1050 && rc_roll < 1050){
		if(!arm_progress) {
			arm_timeout = timeout_from_now(1000000); 
			arm_progress = 1; 
		} else if(timeout_expired(arm_timeout)){
			set_pin(LED_PIN, 0); 
			armed = 0; 
			arm_progress = 0; 
		}
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
	kdebug("RC: [%d, %d, %d, %d]\n", 
		rc_throttle, 
		rc_yaw, 
		rc_pitch, 
		rc_roll, 
		rc_mode); 
	kprintf("THR: [%d, %d, %d, %d]\n", 
		(armed)?thr[0]:MINCOMMAND, 
		(armed)?thr[1]:MINCOMMAND, 
		(armed)?thr[2]:MINCOMMAND, 
		(armed)?thr[3]:MINCOMMAND);
		
	
	
}


