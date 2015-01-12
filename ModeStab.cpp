#include "ModeStab.hpp"
#include <kernel.h>
#include "mavlink.h"

// defualts

#define STAB_PID_KP 4.5 // change 0 - 50
#define STAB_PID_KI 0
#define STAB_PID_KD 0
#define STAB_PID_MAX 30.0

#define RATE_PID_KP 0.5 // change 
#define RATE_PID_KI 0.05
#define RATE_PID_KD 0.04
#define RATE_PID_MAX 500.0

ModeStab::ModeStab() : 
	mPID({
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), // pitch
		AC_PID(10, 0, 10, 30), // yaw
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), // roll
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX),
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
	// get normalized gravity vector
	/*glm::vec3 nacc = glm::normalize(raw_gravity);
	//float pp = 0, py = 0, pr = 0; 
	float ap = 0, ay = 0, ar = 0; 
	
	ap = glm::degrees(::atan2(nacc.y , nacc.z )); 
	ay = 0; 
	ar = glm::degrees(::atan2(nacc.x , nacc.z )); 
	
	float gp, gy, gr; 
	gp = raw_omega.x * dt; //0.9 * gp + gyr.x * 0.1; 
	gy = raw_omega.z * dt; //0.9 * gy + gyr.y * 0.1; 
	gr = raw_omega.y * dt; //0.9 * gr + gyr.z * 0.1; 
	
	//fuse accelerometer and gyroscope into ypr using comp filter
	mAccPitch = 0.98 * (mAccPitch + gp) + 0.02 * ap; // needs to be + here for our conf front/back/left/right
	//mAccYaw 	= 0.98 * (mAccYaw 	- gy) + ; 
	// integrate gyro directly, but filter out low level noise
	mAccYaw 	+= (abs(raw_omega.z) > 2)?(gy):0; 
	mAccRoll 	= 0.98 * (mAccRoll 	- gr) + 0.02 * ar; 
	*/
	
	float rcp = -map(rc.pitch, 1000, 2000, -25, 25); //(pitch - 1500.0); 
	float rcr = -map(rc.roll, 1000, 2000, -25, 25); //(roll - 1500.0); 
	float rcy = -map(rc.yaw, 1000, 2000, -50, 50); //(yaw - 1500.0); 
	
	// control target yaw using control sticks
	mTargetYaw = mTargetYaw + rcy * dt; 
	
	// calculate desired rotation rate in degrees / sec
	float sp = constrain(mPID[PID_STAB_PITCH].get_pid(rcp - pitch, dt), -250, 250); 
	float sr = constrain(mPID[PID_STAB_ROLL].get_pid(rcr 	- roll, dt), -250, 250); 
	float sy = 0; //constrain(mPID[PID_STAB_YAW].get_pid(gy, dt), -360, 360); 
	
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
