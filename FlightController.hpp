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
#pragma once

#include <math.h>

#define GLM_FORCE_RADIANS

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#include "SensorProvider.hpp"
#include "PID.hpp"

//typedef unsigned short uint16_t;
typedef signed short ivalue; 
typedef unsigned short uivalue; 
/*typedef glm::i16vec4 ivec4;
typedef glm::i16vec3 ivec3;*/

#define MAX_UVALUE(a) (2^(sizeof(a)*8))
#define MAX_IVALUE (MAX_VALUE(ivalue)/2)
#define MIN_IVALUE (-MAX_IVALUE)
#define MAX_UIVALUE (MAX_VALUE(uivalue))

enum {
	PID_STAB_PITCH, 
	PID_STAB_YAW, 
	PID_STAB_ROLL, 
	PID_RATE_PITCH, 
	PID_RATE_YAW, 
	PID_RATE_ROLL,
	PID_COUNT
}; 

/**
 * Flight controller that accepts inputs from RC control and calculates
 * motor thrust.
 */
 
class FlightController {
public:
	FlightController();
	void 	SetBoardInterface(struct fc_quad_interface *board){
		mBoard = board;
	}
	
	void reset(); 
	
	virtual void update(timestamp_t dt);
protected:
	struct fc_quad_interface *mBoard;
	
	AC_PID mPID[PID_COUNT]; 
	
	// outputs
	//glm::i16vec4 mThrottle;
	//glm::vec3 mAcc, mGyr, mMag; 
	//float mAltitude, mPressure, mTemperature; 
	
	//value mTargetYaw;
	
	/*float mRCThrottle, mRCPitch, mRCYaw, mRCRoll;

	float mAltErrorInt, mAltPrevError; */

}; 
