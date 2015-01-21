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

#include <kernel.h>
#include "ArmSwitch.hpp"

bool ArmSwitch::TryArm(const RCValues &rc){
	if(!mArmTimeout) mArmTimeout = timestamp_now(); 
	
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
