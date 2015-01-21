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

#include <stdio.h>

extern "C" {
#include <math.h>
}

#include <kernel.h>

#ifdef CONFIG_NATIVE
#include "simulator/sim_kernel.h"
#endif

#include "FlightController.hpp"

/*
#define ALT_PID_KP 1.8 // altitude 
#define ALT_PID_KI 0.018
#define ALT_PID_KD 30.0
#define ALT_PID_MAX 20.0
*/

extern "C" double atan2(double x, double y); 

FlightController::FlightController(){
	mMode = MODE_STABILIZE; 
	mAccYaw = 0; 
	//mAccPitch = 0; 
	//mAccRoll = 0; 
}

void FlightController::SetPIDValues(
		const pid_values_t &stab_yaw, 
		const pid_values_t &stab_pitch, 
		const pid_values_t &stab_roll, 
		const pid_values_t &rate_yaw, 
		const pid_values_t &rate_pitch, 
		const pid_values_t &rate_roll){
	
	mStabCtrl.SetPIDValues(
		stab_yaw,
		stab_pitch, 
		stab_roll, 
		rate_yaw, 
		rate_pitch, 
		rate_roll); 
}

void FlightController::Reset(){
	//mAltHoldCtrl.SetAltitude(raw_altitude); 
	mStabCtrl.Reset(); 
}


ThrottleValues FlightController::ComputeThrottle(float dt, const RCValues &rc,  
		const glm::vec3 &acc, const glm::vec3 &gyr, const glm::vec3 &mag,
		float altitude
	){
	// frame time in seconds, prevent zero
	if(dt < 0.000001) dt = 0.000001; 
	
	uint16_t exp_thr = rc.throttle; 
	/*
	// use log curve for throttle (adjust for values between 1000 to 2000)
	if(rc.throttle >= 1000 && rc.throttle <= 2000){
		// map 1000-2000 range into 0-1; 
		float x = ((float)rc.throttle-1000.0)/1000.0; 
		// calculate curve and map back to 1000-2000
		exp_thr = (uint16_t)(2000.0+(log(x)) * 500.0); // 1+log(x)/2
	}*/
	
	exp_thr = map(rc.throttle, 0, 2000, 0, 1700); 
	
	glm::vec3 nacc = glm::normalize(acc);
	
	/// maybe this should be rate directly from gyro? 
	float rpitch = 	(gyr.x ); //0.9 * gp + gyr.x * 0.1; 
	float ryaw = 		(gyr.z ); //0.9 * gy + gyr.y * 0.1; 
	float rroll = 	(gyr.y ); //0.9 * gr + gyr.z * 0.1; 
	
	// in radians
	float ap = glm::degrees(::atan2(nacc.y , nacc.z )); 
	float ar = -glm::degrees(::atan2(nacc.x , nacc.z )); 
	
	//fuse accelerometer and gyroscope into ypr using comp filter
	//mAccPitch = 0.99 * (mAccPitch + rpitch) + 0.01 * ap; // needs to be + here for our conf front/back/left/right
	//mAccRoll 	= 0.99 * (mAccRoll 	+ rroll) + 0.01 * ar; 
	// integrate gyro directly, but filter out low level noise
	mAccYaw 	+= (abs(gyr.z) > 2)?(ryaw):0; 
	
	float yaw = mAccYaw; //glm::degrees(atan2(mag.y, mag.x)); //mAccYaw; 
	float pitch = mKalmanPitch.UpdateAngle(ap, gyr.x, dt); 
	float roll = mKalmanRoll.UpdateAngle(ar, gyr.y, dt); 
	// when we accumulate readings then we SHOULD use sliced gyro data
	/*float pitch = mAccPitch = 
		0.99 * (mAccPitch + gyr.x * dt) + 0.01 * ap;
	float roll = mAccRoll = 
		0.99 * (mAccRoll 	+ gyr.y * dt) + 0.01 * ar; 
	*/
	ThrottleValues stab = mStabCtrl.ComputeThrottle(
		dt, rc, 
		yaw, pitch, roll, 
		ryaw, rpitch, rroll
	); 
	ThrottleValues althold = mAltHoldCtrl.ComputeThrottle(
		dt, rc, 
		altitude
	); 
	
	ThrottleValues throttle = ThrottleValues(MINCOMMAND); 
	
	// compute final motor throttle
	if(mMode == MODE_ALT_HOLD){
		throttle = 1000 + althold + stab; 
	} else if(mMode == MODE_STABILIZE){
		throttle = ThrottleValues(exp_thr) + stab; 
	}
	
	for(int c = 0; c < 4; c++){
		throttle[c] = constrain(throttle[c], PWM_MIN, PWM_MAX); 
	}
	
	return throttle; 
}
