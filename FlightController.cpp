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

// Globals
#define STAB_PID_KP 2.1 // change 0 - 50
#define STAB_PID_KI 0.4
#define STAB_PID_KD 0.8
#define STAB_PID_MAX 30.0

#define RATE_PID_KP 1.0 // change 
#define RATE_PID_KI 0.0
#define RATE_PID_KD 0.0
#define RATE_PID_MAX 10.0

extern "C" double atan2(double x, double y); 

FlightController::FlightController() : 
	mPID({
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), 
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), 
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX)
	}),
	mThrottle(0, 0, 0, 0){
	
}

void FlightController::reset(){
	for(int c = 0; c < 4; c++){
		mPID[c].reset_I(); 
	}
}

void FlightController::update(
		int16_t rc_thr, 
		int16_t rc_yaw, 
		int16_t rc_pitch, 
		int16_t rc_roll, 
		int16_t mode, timeout_t udt){
	
	float dt = udt * 0.000001;
	
	glm::vec3 nacc = glm::normalize(mAcc);
	static float pp = 0, py = 0, pr = 0; 
	static float ap = 0, ay = 0, ar = 0; 
	
	ap = glm::degrees(::atan2(nacc.z , nacc.y )); 
	ay = 0; 
	ar = glm::degrees(::atan2(nacc.x , nacc.y )); 
	
	/*float gp = glm::degrees(mGyr.x * 0.01f); 
	float gy = glm::degrees(mGyr.y * 0.01f); 
	float gr = glm::degrees(mGyr.z * 0.01f); */
	static float gp, gy, gr; 
	gp = mGyr.x * dt; //0.9 * gp + mGyr.x * 0.1; 
	gy = mGyr.y * dt ; //0.9 * gy + mGyr.y * 0.1; 
	gr = mGyr.z * dt ; //0.9 * gr + mGyr.z * 0.1; 
	
	pp = 0.98 * (pp + gp) + 0.02 * ap; 
	py = 0.98 * (py - gy) + 0.02 * ay; 
	pr = 0.98 * (pr - gr) + 0.02 * ar; 
	
	float rcp = -map(rc_pitch, 1000, 2000, -45, 45); //(pitch - 1500.0); 
	float rcr = -map(rc_roll, 1000, 2000, -45, 45); //(roll - 1500.0); 
	float rcy = -map(rc_yaw, 1000, 2000, -150, 150); //(yaw - 1500.0); 
	
	float sp = constrain(mPID[PID_STAB_PITCH].get_pid(rcp - pp, dt), -250, 250); 
	float sr = constrain(mPID[PID_STAB_ROLL].get_pid(rcr - pr, dt), -250, 250); 
	float sy = constrain(mPID[PID_STAB_YAW].get_pid((rcy)?(rcy):py, dt), -360, 360); 
	
	float rp = constrain(mPID[PID_RATE_PITCH].get_pid(sp - gp, dt), -500, 500); 
	float rr = constrain(mPID[PID_RATE_ROLL].get_pid(sr - gr, dt), -500, 500); 
	float ry = constrain(mPID[PID_RATE_YAW].get_pid(sy - gy, dt), -500, 500); 
	
	mThrottle = glm::i16vec4(
		constrain(rc_thr					+ rp - ry, PWM_MIN, PWM_MAX), 
		constrain(rc_thr					- rp - ry, PWM_MIN, PWM_MAX),
		constrain(rc_thr + rr					 + ry, PWM_MIN, PWM_MAX), 
		constrain(rc_thr - rr					 + ry, PWM_MIN, PWM_MAX) 
	); 
	
	//PORTC &= ~_BV(2); 
	
	kdebug("RP: %-4d, RR: %-4d, RY: %-4d ", 
		(int16_t)(rc_pitch ), (int16_t)(rc_roll ), (int16_t)(rc_yaw )); 
	kdebug("RCP: %-4d, RCR: %-4d, RCY: %-4d ", 
		(int16_t)(rcp ), (int16_t)(rcr ), (int16_t)(rcy )); 
	kdebug("PP: %-4d, PR: %-4d, PY: %-4d ", 
		(int16_t)(pp * 100), (int16_t)(pr * 100), (int16_t)(py * 100)); 
	kdebug("AP: %-4d, AR: %-4d, AY: %-4d ", 
		(int16_t)(ap * 100), (int16_t)(ar * 100), (int16_t)(ay * 100)); 
	kdebug("A: %-4d, A: %-4d, A: %-4d ", 
		(int16_t)(mAcc.x * 100), (int16_t)(mAcc.y * 100), (int16_t)(mAcc.z * 100)); 
	kdebug("GP: %-4d, GR: %-4d, GY: %-4d ", 
		(int16_t)(gp * 100), (int16_t)(gr * 100), (int16_t)(gy * 100)); 
	kdebug("SP: %-4d, SR: %-4d, SY: %-4d ", 
		(int16_t)(sp), (int16_t)(sr), (int16_t)(sy)); 
	kdebug("RP: %-4d, RR: %-4d, RY: %-4d\n", 
		(int16_t)(rp), (int16_t)(rr), (int16_t)(ry)); 
	kprintf("THR: [%-4d, %-4d, %-4d, %-4d]\n", 
		mThrottle[0], 
		mThrottle[1], 
		mThrottle[2], 
		mThrottle[3]
	); 
	/*
	glm::vec4 dth; 
	glm::vec3 target(0.0f, 1.0f, 0.0f); 
	glm::vec3 nacc = glm::normalize(acc); 
	
  static glm::vec3 pyr(0.0f, 0.0f, 0.0f); 
	
	glm::vec3 npyr(
		glm::degrees(atan2(nacc.z , nacc.y )), 
		0.0f, 
		glm::degrees(atan2(nacc.x , nacc.y ))); 
	// filter acc
	pyr = (pyr + mGyr * dt * 0.01f) * 0.8f + npyr * 0.2f; 
	
	glm::vec3 set = glm::vec3(
		mPID[0].get_pid(tp * 0.01 + pyr.x, dt) * 10.0f, 
		mPID[1].get_pid(ty * 0.01 + pyr.y, dt) * 10.0f, 
		mPID[2].get_pid(tr * 0.01 + pyr.z, dt) * 10.0f
	);
	
	kdebug("PITCH: %04d, YAW: %04d, ROLL: %04d\n", 
		(int)(set.x * 100), 
		(int)(set.y * 100), 
		(int)(set.z * 100)); 
	
	mThrottle = glm::i16vec4(
		thr 				- set.x + set.y, 
		thr					+ set.x + set.y,
		thr - set.z 			  - set.y, 
		thr + set.z					- set.y 
	); */
}
