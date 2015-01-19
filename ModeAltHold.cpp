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
