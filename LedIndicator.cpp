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
#include "LedIndicator.hpp"
#include "types.hpp"

void LedIndicator::Update(){
	if(led_value && blink_low_delay > 0 && 
		timestamp_expired(blink_timeout)){
		led_value = 0; 
		gpio_clear(FC_LED_PIN); 
		blink_timeout = timestamp_from_now_us(blink_low_delay * 1000L); 
	} else if(!led_value && blink_high_delay > 0 &&
		timestamp_expired(blink_timeout)){
		led_value = 1; 
		gpio_set(FC_LED_PIN); 
		blink_timeout = timestamp_from_now_us(blink_high_delay * 1000); 
	}
}
