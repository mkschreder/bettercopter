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

/** 
Implements a wrapper around the flight controller led adding 
support for various ways to signal user about events. 
*/
class LedIndicator {
public:
	LedIndicator():led_value(0), blink_high_delay(0), 
		blink_low_delay(0), blink_timeout(0){
	}
	void SlowBlink(){blink_high_delay = 1000; blink_low_delay = 1000; }
	void FastBlink(){blink_high_delay = 200; blink_low_delay = 200; }
	void On(){blink_high_delay = 1000; blink_low_delay = 0;}
	void Off(){blink_high_delay = 0; blink_low_delay = 1000;}
	/// updates the state of the led
	void Update(); 
private:	
	/// current led value 
	uint8_t led_value; 
	/// delay in ms of led being lit
	uint32_t blink_high_delay; 
	/// delay in ms of led being dark
	uint32_t blink_low_delay; 
	/// timeout monitor
	timestamp_t blink_timeout; 
}; 
