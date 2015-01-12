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
	
	AC_PID mPID[STAB_PID_COUNT];
	float mTargetYaw;  
}; 

