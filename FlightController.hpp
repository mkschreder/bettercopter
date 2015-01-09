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
	void 	SetBoardInterface(struct fc_quad_interface *board){
		mBoard = board;
	}
	
	void reset(); 
	
	virtual void update(timestamp_t dt);
protected:
	struct fc_quad_interface *mBoard;
	
	// flight control modules
	ModeAltHold mAltHoldCtrl; 
	ModeStab 		mStabCtrl; 
	FlightMode	mMode; 
	
	bool mArmed, mArmInProgress; 
	timestamp_t mArmTimeout; 
	
	// acceleration change per second
	glm::vec3 mAccPrev; 
protected: 
	void _CheckArm(const uint16_t &rc_thr, const uint16_t &rc_roll, const float &def_alt); 
}; 
