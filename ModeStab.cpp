#include "ModeStab.hpp"
#include <kernel.h>
#include "mavlink.h"

// defualts

ModeStab::ModeStab() : 
	mPID({
		AC_PID(0, 0, 0, 0), // pitch
		AC_PID(0, 0, 0, 0), // yaw
		AC_PID(0, 0, 0, 0), // roll
		AC_PID(0, 0, 0, 0), 
		AC_PID(0, 0, 0, 0), 
		AC_PID(0, 0, 0, 0),
		//AC_PID(ALT_PID_KP, 	ALT_PID_KI,  ALT_PID_KD, 	ALT_PID_MAX)
	}){
		
}

void ModeStab::SetPIDValues(
	const pid_values_t &stab_yaw, 
	const pid_values_t &stab_pitch, 
	const pid_values_t &stab_roll, 
	const pid_values_t &rate_yaw, 
	const pid_values_t &rate_pitch, 
	const pid_values_t &rate_roll){
	mPID[PID_STAB_YAW] = AC_PID(stab_yaw.p, stab_yaw.i, stab_yaw.d, stab_yaw.max_i); 
	mPID[PID_STAB_PITCH] = AC_PID(stab_pitch.p, stab_pitch.i, stab_pitch.d, stab_pitch.max_i); 
	mPID[PID_STAB_ROLL] = AC_PID(stab_roll.p, stab_roll.i, stab_roll.d, stab_roll.max_i); 
	mPID[PID_RATE_YAW] = AC_PID(rate_yaw.p, rate_yaw.i, rate_yaw.d, rate_yaw.max_i); 
	mPID[PID_RATE_PITCH] = AC_PID(rate_pitch.p, rate_pitch.i, rate_pitch.d, rate_pitch.max_i); 
	mPID[PID_RATE_ROLL] = AC_PID(rate_roll.p, rate_roll.i, rate_roll.d, rate_roll.max_i); 
}

void ModeStab::Reset(){
	for(int c = 0; c < STAB_PID_COUNT; c++){
		mPID[c].reset_I(); 
	}
}

ThrottleValues ModeStab::ComputeThrottle(float dt, const RCValues &rc, 
	float yaw, float pitch, float roll, 
	float omega_yaw, float omega_pitch, float omega_roll){
	
	float rcp = -map(rc.pitch, 1000, 2000, -10, 10); //(pitch - 1500.0); 
	float rcr = -map(rc.roll, 1000, 2000, -10, 10); //(roll - 1500.0); 
	float rcy = -map(rc.yaw, 1000, 2000, -50, 50); //(yaw - 1500.0); 
	
	// control target yaw using control sticks
	mTargetYaw = mTargetYaw + rcy * dt; 
	
	// calculate desired rotation rate in degrees / sec
	float sp = constrain(mPID[PID_STAB_PITCH].get_pid(rcp - pitch, dt), -250, 250); 
	float sr = constrain(mPID[PID_STAB_ROLL].get_pid(rcr 	- roll, dt), -250, 250); 
	float sy = constrain(mPID[PID_STAB_YAW].get_pid(rcy, dt), -250, 250); 
	
	// calculate the actual rate based on current gyro rate
	float rp = constrain(mPID[PID_RATE_PITCH].get_pid(sp - omega_pitch, dt), -500, 500); 
	float rr = constrain(mPID[PID_RATE_ROLL].get_pid(sr - omega_roll, dt), -500, 500); 
	float ry = constrain(mPID[PID_RATE_YAW].get_pid(sy - omega_yaw, dt), -500, 500); 
	
	return glm::i16vec4(
		// front
					+ rp + ry,
		// back  
					- rp + ry,
		// left 
		- rr			 - ry,
		// right
		+ rr			 - ry
	); 
}
