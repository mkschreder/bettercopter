#pragma once

#include "types.hpp"
#include "PID.hpp"

class ModeStab {
public:
	ModeStab(); 
	ThrottleValues ComputeThrottle(float dt, const RCValues &rc, const glm::vec3 &raw_gravity, const glm::vec3 &raw_omega); 
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
	float mAccPitch, mAccYaw, mAccRoll; 
	float mTargetYaw;  
}; 

