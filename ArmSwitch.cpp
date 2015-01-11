 
#include <kernel.h>
#include "ArmSwitch.hpp"

bool ArmSwitch::TryArm(const RCValues &rc){
	if(!mArmed && rc.throttle < 1050 && rc.roll > 1700){
		if(!mArmInProgress) {
			mArmTimeout = timestamp_from_now_us(1000000); 
			mArmInProgress = true; 
		} else if(timestamp_expired(mArmTimeout)){
			mArmed = true; 
			mArmInProgress = false; 
			return true; 
		}
	} else if(timestamp_expired(mArmTimeout)){
		mArmInProgress = false; 
	}
	return false; 
}

bool ArmSwitch::TryDisarm(const RCValues &rc){
	if(mArmed && rc.throttle< 1050 && rc.roll < 1100){
		if(!mArmInProgress) {
			mArmTimeout = timestamp_from_now_us(1000000); 
			mArmInProgress = true; 
		} else if(timestamp_expired(mArmTimeout)){
			mArmed = false; 
			mArmInProgress = false; 
			return true; 
		}
	} else if(timestamp_expired(mArmTimeout)){
		mArmInProgress = false; 
	}
	return false; 
}
