#include "ModeAltHold.hpp"
#include <kernel.h>

ModeAltHold::ModeAltHold() : mAccAltitude(0), mTargetAltitude(0) {
	
}
 
ThrottleValues ModeAltHold::ComputeThrottle(float raw_altitude){
	// filter raw altitude value
	mAccAltitude = mAccAltitude + (raw_altitude - mAccAltitude) * 0.2; //0.78 * dacc + 0.22 * alt; 
	
	float dalt = constrain(
		(int16_t)(1100 + (mTargetAltitude - mAccAltitude) * 50.0), 
		1000, 2000); 
	if(mTargetAltitude < 130) mTargetAltitude = 130; 
	
	kdebug("ALTA: %-4ld, DALT: %-4ld, TALT: %-4ld, ", 
		(long)(mAccAltitude * 100),
		(long)(dalt * 100), 
		(long)(mTargetAltitude * 100)); 
	
	return ThrottleValues(dalt); 
}

void ModeAltHold::SetAltitude(float alt){
	mTargetAltitude = alt; 
}

void ModeAltHold::AdjustAltitude(float alt){
	mTargetAltitude += alt; 
}
