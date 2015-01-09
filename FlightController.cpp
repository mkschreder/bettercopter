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

#include <stdio.h>

extern "C" {
#include <math.h>
}

#include <kernel.h>

#ifdef CONFIG_NATIVE
#include "simulator/sim_kernel.h"
#endif

#include "FlightController.hpp"

#define ALT_PID_KP 1.8 // altitude 
#define ALT_PID_KI 0.018
#define ALT_PID_KD 30.0
#define ALT_PID_MAX 20.0

extern "C" double atan2(double x, double y); 

FlightController::FlightController(){
	mBoard = 0; 
	mArmInProgress = mArmed = false; 
	mArmTimeout = 0; 
	mMode = MODE_STABILIZE; 
}

void FlightController::reset(){
	
}

void FlightController::_CheckArm(const uint16_t &rc_thr, const uint16_t &rc_roll, const float &raw_altitude){
	if(!mArmed && rc_thr < 1050 && rc_roll > 1700){
		if(!mArmInProgress) {
			mArmTimeout = timestamp_from_now_us(1000000); 
			mArmInProgress = true; 
		} else if(timestamp_expired(mArmTimeout)){
			kdebug("ARMED!\n"); 
			mAltHoldCtrl.SetAltitude(raw_altitude); 
			mStabCtrl.Reset(); 
			gpio_set(FC_LED_PIN); 
			mArmed = true; 
			mArmInProgress = false; 
		}
	} else if(mArmed && rc_thr < 1050 && rc_roll < 1100){
		if(!mArmInProgress) {
			mArmTimeout = timestamp_from_now_us(1000000); 
			mArmInProgress = true; 
		} else if(timestamp_expired(mArmTimeout)){
			gpio_clear(FC_LED_PIN); 
			mArmed = false; 
			mArmInProgress = false; 
		}
	} else if(timestamp_expired(mArmTimeout)){
		mArmInProgress = false; 
	}
}

void FlightController::update(timestamp_t udt){
	RCValues rc; 
	glm::vec3 acc, gyr, mag;
	
	if(!mBoard) {
		kprintf("No board!\n"); 
		return;
	}
	
	// frame time in seconds, prevent zero
	float dt = udt * 0.000001; if(dt < 0.000001) dt = 0.000001; 
	
	mBoard->read_receiver(mBoard, &rc.throttle, &rc.yaw, &rc.pitch, &rc.roll, &rc.aux0, &rc.aux1);
	
	mBoard->read_accelerometer(mBoard, &acc.x, &acc.y, &acc.z);
	mBoard->read_gyroscope(mBoard, &gyr.x, &gyr.y, &gyr.z);
	
	uint16_t exp_thr = rc.throttle; 
	
	// use log curve for throttle (adjust for values between 1000 to 2000)
	if(rc.throttle >= 1000 && rc.throttle <= 2000){
		// map 1000-2000 range into 0-1; 
		float x = ((float)rc.throttle-1000.0)/1000.0; 
		// calculate curve and map back to 1000-2000
		exp_thr = (uint16_t)(2000.0+(log(x)) * 500.0); // 1+log(x)/2
	}
	
	// get altitude and store it into a filtered accumulator
	float altitude = mBoard->read_altitude(mBoard); 
	
	_CheckArm(rc.throttle, rc.roll, altitude); 
	
	if(mArmed && rc.throttle > 1050) {
		if(rc.throttle < 1300){
			mAltHoldCtrl.AdjustAltitude(-0.1); 
		} else if(rc.throttle > 1700){
			mAltHoldCtrl.AdjustAltitude(0.1); 
		}
	}
	
	ThrottleValues stab = mStabCtrl.ComputeThrottle(dt, rc, acc, gyr); 
	ThrottleValues althold = mAltHoldCtrl.ComputeThrottle(altitude); 
	
	ThrottleValues throttle = ThrottleValues(MINCOMMAND); 
	
	// compute final motor throttle
	if(!mArmed || (rc.throttle <= 1050)){ // prevent spin when arming!
		throttle = ThrottleValues(MINCOMMAND); 
	} else if(mMode == MODE_ALT_HOLD){
		throttle = althold + stab; 
	} else if(mMode == MODE_STABILIZE){
		throttle = ThrottleValues(exp_thr) + stab; 
	}
	
	for(int c = 0; c < 4; c++){
		throttle[c] = constrain(throttle[c], PWM_MIN, PWM_MAX); 
	}
	
	mBoard->write_motors(mBoard, 
			throttle[0], throttle[1], throttle[2], throttle[3]);
	
	kdebug("RP: %-4d, RR: %-4d, RY: %-4d ", 
		(int16_t)(rc.pitch ), (int16_t)(rc.roll ), (int16_t)(rc.yaw )); 
	
	kdebug("A: %-4d, A: %-4d, A: %-4d ", 
		(int16_t)(acc.x * 100), (int16_t)(acc.y * 100), (int16_t)(acc.z * 100)); 

	kdebug("RC: [%-4d, %-4d, %-4d, %-4d] ", 
		(uint16_t)rc.throttle, 
		(uint16_t)rc.yaw, 
		(uint16_t)rc.pitch, 
		(uint16_t)rc.roll, 
		(uint16_t)rc.aux0); 
		
	kprintf("THR: [%-4d, %-4d, %-4d, %-4d]\n", 
		throttle[0], 
		throttle[1], 
		throttle[2], 
		throttle[3]
	); 
}
