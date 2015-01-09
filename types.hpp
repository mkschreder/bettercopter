#pragma once

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#define GLM_FORCE_RADIANS

//#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

typedef glm::i16vec4 ThrottleValues; 

struct RCValues {
	RCValues():throttle(0), yaw(0), pitch(0), roll(0), aux0(0), aux1(0){}
	/*RCValues(uint16_t thr, uint16_t yaw, uint16_t pitch, uint16_t roll):
		throttle(thr), yaw(yaw), pitch(pitch), roll(roll) {
			
	}*/
	uint16_t throttle, yaw, pitch, roll, aux0, aux1; 
}; 

#define frame_log(fmt, ...) {} //kprintf(fmt, ##__VA_ARGS__) 
