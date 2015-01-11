#pragma once 

#include "types.hpp"

class ArmSwitch {
public: 
	ArmSwitch():mArmed(false), mArmInProgress(false), mArmTimeout(0){
		
	}
	/// looks at the rc values and returns true only when going from 
	/// disarmed to armed state. 
	bool TryArm(const RCValues &rc); 
	/// looks at the rc values and returns true only when going from 
	/// armed to disarmed state
	bool TryDisarm(const RCValues &rc); 
	/// checks if armed and returns true if armed
	inline bool IsArmed(){
		return mArmed; 
	}
private: 
	bool mArmed, mArmInProgress; 
	timestamp_t mArmTimeout; 
}; 
