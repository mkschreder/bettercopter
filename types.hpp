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
#pragma once

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#define GLM_FORCE_RADIANS

//#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#define fc_init() mwii_init()
#define fc_process_events() mwii_process_events()
#define fc_get_interface() mwii_get_fc_quad_interface()
#define FC_LED_PIN MWII_LED_PIN

typedef enum {
	COPTER_STATE_BOOT=1, /* System is booting up. | */
	COPTER_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
	COPTER_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
	COPTER_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
	COPTER_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
	COPTER_STATE_EMERGENCY=6
} copter_state_t; 

typedef glm::i16vec4 ThrottleValues; 

typedef struct {
	float p, i, d, max_i; 
} pid_values_t; 

struct RCValues {
	RCValues():throttle(0), yaw(0), pitch(0), roll(0), aux0(0), aux1(0){}
	/*RCValues(uint16_t thr, uint16_t yaw, uint16_t pitch, uint16_t roll):
		throttle(thr), yaw(yaw), pitch(pitch), roll(roll) {
			
	}*/
	uint16_t throttle, yaw, pitch, roll, aux0, aux1; 
}; 

#define frame_log(fmt, ...) {} //kprintf(fmt, ##__VA_ARGS__) 
