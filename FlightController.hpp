/*
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
#pragma once

#include <math.h>

#include "types.hpp"
#include "SensorProvider.hpp"
#include "PID.hpp"
#include "ModeStab.hpp"
#include "ModeAltHold.hpp"

//typedef unsigned short uint16_t;
typedef signed short ivalue; 
typedef unsigned short uivalue; 
/*typedef glm::i16vec4 ivec4;
typedef glm::i16vec3 ivec3;*/

#define MAX_UVALUE(a) (2^(sizeof(a)*8))
#define MAX_IVALUE (MAX_VALUE(ivalue)/2)
#define MIN_IVALUE (-MAX_IVALUE)
#define MAX_UIVALUE (MAX_VALUE(uivalue))

/**
 * Flight controller that accepts inputs from RC control and calculates
 * motor thrust.
 */
 
class FlightController {
public:
	enum FlightMode {
		MODE_STABILIZE, 
		MODE_ALT_HOLD
	}; 
	
	FlightController();
	//void ComputeAngles(const glm::vec3 &raw_acc, float *yaw, float *pitch, float *roll); 
	//void ComputeOmega(const glm::vec3 &raw_gyr, float *yaw_rate, float *pitch_rate, float *roll_rate); 
	
	void Reset(); 
	
	ThrottleValues ComputeThrottle(
		float dt, 
		const RCValues &rc,  
		const glm::vec3 &acc, 
		const glm::vec3 &gyr, 
		const glm::vec3 &mag,
		float altitude
	);
	
	void SetPIDValues(
		const pid_values_t &stab_yaw, 
		const pid_values_t &stab_pitch, 
		const pid_values_t &stab_roll, 
		const pid_values_t &rate_yaw, 
		const pid_values_t &rate_pitch, 
		const pid_values_t &rate_roll); 
protected:
	//struct fc_quad_interface *mBoard;
	
	// flight control modules
	ModeAltHold mAltHoldCtrl; 
	ModeStab 		mStabCtrl; 
	FlightMode	mMode; 
	
	float mAccPitch, mAccYaw, mAccRoll; 
	//bool mArmed, mArmInProgress; 
	//timestamp_t mArmTimeout; 
	
	// acceleration change per second
	//glm::vec3 mAccPrev; 
}; 
