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
/** Altitude hold controller 
The controller PID determines how many units to increase throttle in 
order to quickly reach desired altitude. If set to 50 then for 1 meter 
difference we will increase throttle 50 units. 

The I term is particularly important because without it we may never 
reach target altitude becuase of weight. 

Normally, at ground level we have about 8cm altitude (0.08m). This 
means that we will have throttle a little below hover point. When we are
down at the ground level however (below 0.15m), we need to gradually 
increase throttle until the copter lifts off. Once we are in the air, 
we can increase or decrease the set point to hover. 
*/
#include "ModeAltHold.hpp"
#include <kernel.h>

#define MIN_HOVER_ALTITUDE 0.15 // meters
#define MAX_HOVER_ALTITUDE 2.0

#define ALTITUDE_STEP 0.2 // 20cm / sec
#define THROTTLE_STEP 10.0 // 10 units / sec
#define MAX_BASE_THROTTLE 2000

ModeAltHold::ModeAltHold() : mBaseThrottle(0), mTargetAltitude(0){
	mAltPID = PID(150, 2, 0, 50); 
}

ThrottleValues ModeAltHold::ComputeThrottle(float dt, const RCValues &rc, float raw_altitude){
	// automatically spin up propellers if we are not on minimal throttle 
	// and we have not reached takeoff
	if(
		rc.throttle > 1050 && 
		raw_altitude > 0 && 
		raw_altitude < MIN_HOVER_ALTITUDE
	){
		mBaseThrottle = constrain(
			mBaseThrottle + THROTTLE_STEP * dt, 
			0, MAX_BASE_THROTTLE); 
	} 
	// if powered off then gradually stop propellers
	else if(rc.throttle < 1050){
		mBaseThrottle = constrain(
			mBaseThrottle - THROTTLE_STEP * dt, 
			0, MAX_BASE_THROTTLE); 
	}
	// if throttle goes beyond "dead zone" then we move upwards
	if(rc.throttle > 1800) {
		mTargetAltitude = constrain(
			mTargetAltitude + ALTITUDE_STEP * dt, 
			0, MAX_HOVER_ALTITUDE); 
	} 
	// if blow "dead zone" then downwards"
	else if(rc.throttle < 1200){
		mTargetAltitude = constrain(
			mTargetAltitude - ALTITUDE_STEP * dt, 
			0, MAX_HOVER_ALTITUDE); 
	}
	// adjust throttle for altitude error 
	// increase if we are below target and decrease if above
	float thr = mBaseThrottle; 
	// limit error so that it never gets out of hand
	float err = constrain(mTargetAltitude - raw_altitude, -50, 50); 
	thr += mAltPID.get_pid(err, dt); 
	
	return ThrottleValues(thr); 
}

