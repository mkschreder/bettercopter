#pragma once

#include "types.hpp"

class ModeAltHold {
public:
	ModeAltHold(); 
	ThrottleValues ComputeThrottle(float raw_altitude); 
	void 			AdjustAltitude(float units); 
	void 			SetAltitude(float target); 
private:
	float mAccAltitude; // accumulated/accelerometer corrected altitude
	float mTargetAltitude; // target altitude
};
