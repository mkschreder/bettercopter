 
#include <kernel.h>
#include "LedIndicator.hpp"

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
