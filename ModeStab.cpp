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

#include "ModeStab.hpp"
#include <kernel.h>
#include "mavlink.h"

// defualts

ModeStab::ModeStab() : 
	mPID({
		PID(0, 0, 0, 0), // pitch
		PID(0, 0, 0, 0), // yaw
		PID(0, 0, 0, 0), // roll
		PID(0, 0, 0, 0), 
		PID(0, 0, 0, 0), 
		PID(0, 0, 0, 0),
		//PID(ALT_PID_KP, 	ALT_PID_KI,  ALT_PID_KD, 	ALT_PID_MAX)
	}){
		
}

void ModeStab::SetPIDValues(
	const pid_values_t &stab_yaw, 
	const pid_values_t &stab_pitch, 
	const pid_values_t &stab_roll, 
	const pid_values_t &rate_yaw, 
	const pid_values_t &rate_pitch, 
	const pid_values_t &rate_roll){
	mPID[PID_STAB_YAW] 		= PID(stab_yaw.p, stab_yaw.i, stab_yaw.d, stab_yaw.max_i); 
	mPID[PID_STAB_PITCH] 	= PID(stab_pitch.p, stab_pitch.i, stab_pitch.d, stab_pitch.max_i); 
	mPID[PID_STAB_ROLL] 	= PID(stab_roll.p, stab_roll.i, stab_roll.d, stab_roll.max_i); 
	mPID[PID_RATE_YAW] 		= PID(rate_yaw.p, rate_yaw.i, rate_yaw.d, rate_yaw.max_i); 
	mPID[PID_RATE_PITCH] 	= PID(rate_pitch.p, rate_pitch.i, rate_pitch.d, rate_pitch.max_i); 
	mPID[PID_RATE_ROLL] 	= PID(rate_roll.p, rate_roll.i, rate_roll.d, rate_roll.max_i); 
}

void ModeStab::Reset(){
	for(int c = 0; c < STAB_PID_COUNT; c++){
		mPID[c].reset_I(); 
	}
}

ThrottleValues ModeStab::ComputeThrottle(float dt, const RCValues &rc, 
	float yaw, float pitch, float roll, 
	float omega_yaw, float omega_pitch, float omega_roll){
	
	float rcp = -map(rc.pitch, 1000, 2000, -25, 25); //(pitch - 1500.0); 
	float rcr =  map(rc.roll, 1000, 2000, -25, 25); //(roll - 1500.0); 
	float rcy = -map(rc.yaw, 1000, 2000, -50, 50); //(yaw - 1500.0); 
	
	// calculate desired rotation rate in degrees / sec
	float sp = constrain(mPID[PID_STAB_PITCH].get_pid(rcp - pitch, dt), -250, 250); 
	float sr = constrain(mPID[PID_STAB_ROLL].get_pid(rcr - roll, dt), -250, 250); 
	float sy = constrain(mPID[PID_STAB_YAW].get_pid(wrap_180(rcy - mTargetYaw), dt), -250, 250); 
	
	if(abs(rc.yaw - 1500) > 10){
		sy = rcy * 0.01;  // this tells us maybe omega should be in degrees per second? 
		mTargetYaw = yaw; 
		//mTargetYaw = mTargetYaw + rcy * dt; 
	} 
	
	// calculate the actual rate based on current gyro rate in degrees
	float rp = constrain(mPID[PID_RATE_PITCH	].get_pid(sp - omega_pitch, dt), -500, 500); 
	float rr = constrain(mPID[PID_RATE_ROLL		].get_pid(sr - omega_roll, dt), -500, 500); 
	float ry = constrain(mPID[PID_RATE_YAW		].get_pid(sy - omega_yaw, dt), -500, 500); 
	
	return glm::i16vec4(
		// front
				+ rp + ry,
		// back  
				- rp + ry,
		// left 
				+ rr - ry,
		// right
				- rr - ry
	); 
}
