
#include <stdio.h>

#include <kernel.h>

#include "FlightController.hpp"

// Globals
#define wrap_pi(x) (x < -M_PI ? x+M_PI*2 : (x > M_PI ? x - M_PI*2: x))

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain(long x, long a, long b){
	if(x < a) return a; 
	if(x > b) return b; 
	return x; 
}

#define STAB_PID_KP 7.5
#define STAB_PID_KI 0.0
#define STAB_PID_KD 0.0
#define STAB_PID_MAX 10.0

#define RATE_PID_KP 1.2
#define RATE_PID_KI 0.0
#define RATE_PID_KD 0.0
#define RATE_PID_MAX 50

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
			
	DDRC |= _BV(2); 
	PORTC |= _BV(2); 
	
	float dt = 0.01f;
	
	glm::vec3 nacc = glm::normalize(mAcc);
	static float pp = 0, py = 0, pr = 0; 
	
	float ap = glm::degrees(atan2(nacc.z , nacc.y )); 
	float ay = 0.0f; 
	float ar = glm::degrees(atan2(nacc.x , nacc.y )); 
	
	/*float gp = glm::degrees(mGyr.x * 0.01f); 
	float gy = glm::degrees(mGyr.y * 0.01f); 
	float gr = glm::degrees(mGyr.z * 0.01f); */
	float gp = mGyr.x * dt; 
	float gy = mGyr.y * dt; 
	float gr = mGyr.z * dt; 
	
	pp = 0.95 * (pp + gp) + 0.05 * ap; 
	py = 0.95 * (py + gp) + 0.05 * ay; 
	pr = 0.95 * (pr + gp) + 0.05 * ar; 
	
	float rcp = -map(rc_pitch, 1000, 2000, -45, 45); //(pitch - 1500.0); 
	float rcr = -map(rc_roll, 1000, 2000, -45, 45); //(roll - 1500.0); 
	float rcy = map(rc_yaw, 1000, 2000, -150, 150); //(yaw - 1500.0); 
	
	float sp = constrain(mPID[PID_STAB_PITCH].get_pid(rcp - pp, dt), -250, 250); 
	float sr = constrain(mPID[PID_STAB_ROLL].get_pid(rcr - pr, dt), -250, 250); 
	float sy = constrain(mPID[PID_STAB_YAW].get_pid(rcy - py, dt), -360, 360); 
	
	float rp = constrain(mPID[PID_RATE_PITCH].get_pid(sp - gp, dt), -500, 500); 
	float rr = constrain(mPID[PID_RATE_ROLL].get_pid(sr - gr, dt), -500, 500); 
	float ry = constrain(mPID[PID_RATE_YAW].get_pid(sy - gy, dt), -500, 500); 
	
	mThrottle = glm::i16vec4(
		constrain(rc_thr					+ rp - ry, PWM_MIN, PWM_MAX), 
		constrain(rc_thr					- rp - ry, PWM_MIN, PWM_MAX),
		constrain(rc_thr + rr					 + ry, PWM_MIN, PWM_MAX), 
		constrain(rc_thr - rr					 + ry, PWM_MIN, PWM_MAX) 
	); 
	
	PORTC &= ~_BV(2); 
	
	kdebug("GP: %d, GR: %d, GY: %d\n", 
		(int16_t)(gp), (int16_t)(gr), (int16_t)(gy)); 
	kdebug("RP: %d, RR: %d, RY: %d\n", 
		(int16_t)(rp), (int16_t)(rr), (int16_t)(ry)); 
		
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
