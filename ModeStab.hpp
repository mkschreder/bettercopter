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

#include "types.hpp"
#include "PID.hpp"

class ModeStab {
public:
	ModeStab(); 
	ThrottleValues ComputeThrottle(float dt, const RCValues &rc, 
		float yaw, float pitch, float roll, 
		float omega_yaw, float omega_pitch, float omega_roll); 
	
	void SetPIDValues(
		const pid_values_t &stab_yaw, 
		const pid_values_t &stab_pitch, 
		const pid_values_t &stab_roll, 
		const pid_values_t &rate_yaw, 
		const pid_values_t &rate_pitch, 
		const pid_values_t &rate_roll); 
	void Reset(); 
private:
	enum {
		PID_STAB_PITCH, 
		PID_STAB_YAW, 
		PID_STAB_ROLL, 
		PID_RATE_PITCH, 
		PID_RATE_YAW, 
		PID_RATE_ROLL,
		STAB_PID_COUNT
	}; 
	
	PID mPID[STAB_PID_COUNT];
	float mTargetYaw;  
}; 

