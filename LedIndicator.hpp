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
