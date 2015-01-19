/*
	This file is part of my quadcopter project.
	https://github.com/mkschreder/bettercopter

	This software is firmware project is free software: you can 
	redistribute it and/or modify it under the terms of the GNU General 
	Public License as published by the Free Software Foundation, either 
	version 3 of the License, or (at your option) any later version.

	This software is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

	Author: Martin K. Schröder
	Email: info@fortmax.se
	Github: https://github.com/mkschreder
*/
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
