#include "sim_kernel.h"
#include <kernel.h>

#define FRONT_PIN PD3
#define RIGHT_PIN PB1
#define LEFT_PIN PB2
#define BACK_PIN PB3

#define PWM_MIN MINCOMMAND
#define PWM_MAX 2000

#define RC_MAX 2000
#define RC_MIN 1000

const static uint16_t rc_defaults[6] = {1000, 1500, 1500, 1500, 1500, 1500}; 

struct board {
	volatile uint16_t pwm[6]; 
	timeout_t rc_time[6]; 
	uint16_t 	rc_value[6]; 
	volatile timeout_t ch_timeout[6]; 
	volatile timeout_t pwm_timeout; 
	timeout_t signal_timeout; 
	timeout_t last_rc_update; 
	uint16_t pwm_pulse_delay_us; 
	volatile uint8_t pwm_lock; 
}; 

static struct board _brd; 
static struct board *brd = &_brd; 

#include "Application.h"
extern Application *App; 

void get_accelerometer(int16_t *x, int16_t *y, int16_t *z){
	glm::vec3 acc; 
	App->getActiveQuad()->getSensorData(&acc, 0, 0, 0, 0, 0); 
	acc = glm::normalize(acc); 
	*x = (int16_t)(acc.x * 32767); 
	*y = (int16_t)(acc.y * 32767); 
	*z = (int16_t)(acc.z * 32767); 
	//mpu6050_getRawData(x, y, z, &gx, &gy, &gz); 
}

void get_gyroscope(float *x, float *y, float *z){
	glm::vec3 v; 
	App->getActiveQuad()->getSensorData(0, &v, 0, 0, 0, 0); 
	//v = v * 100.0f; //glm::normalize(v); 
	*x = v.x; 
	*y = v.y; 
	*z = v.z; 
	//mpu6050_getRawData(&ax, &ay, &az, x, y, z); 
}

void get_magnetometer(int16_t *x, int16_t *y, int16_t *z){
	glm::vec3 v; 
	App->getActiveQuad()->getSensorData(0, 0, &v, 0, 0, 0); 
	v = glm::normalize(v); 
	*x = v.x * 20000; 
	*y = v.y * 20000; 
	*z = v.z * 20000; 
}

void get_altitude(int16_t *alt){
	*alt = 100; 
	//*alt = bmp085_getaltitude(); 
}

void get_pressure(int16_t *pres){
	*pres = 101100; 
	//*pres = bmp085_getpressure() / 10; 
}

void get_temperature(int16_t *temp){
	*temp = 26; 
	//*temp = bmp085_gettemperature(); 
}

void reset_rc(void){
	for(int c = 0; c < 6; c++){
		brd->rc_value[c] = rc_defaults[c]; 
	}
}

extern "C" void brd_init(void){
	brd->pwm_pulse_delay_us = 10500; 
	brd->pwm_lock = 0; 
	brd->pwm_timeout = timeout_from_now(0); 
	
	reset_rc(); 
	
	kprintf("multiwii board!\n"); 
}

extern "C" void brd_process_events(void){
	if(timeout_expired(brd->last_rc_update)){
		reset_rc(); 
	}
}

void set_pin(uint8_t pin, uint16_t value){
	if(pin == LED_PIN) {
		
	} else if(pin >= PWM_FRONT && pin < PWM_COUNT){
		if(value > PWM_MAX) value = PWM_MAX; 
		if(value < PWM_MIN) value = PWM_MIN; 
		
		brd->pwm[pin - PWM_FRONT] = value; 
	} else if(pin >= RC_THROTTLE && pin <= RC_MODE){
		brd->rc_value[pin] = value; 
	} 
}

uint16_t get_pin(uint8_t pin){
	if(pin >= RC_IN0 && pin <= RC_IN4){
		uint16_t val = brd->rc_value[pin - RC_IN0]; 
		if(val > RC_MAX) return RC_MAX; 
		if(val < RC_MIN) return RC_MIN; 
		return val; 
	}	else if(pin >= PWM_FRONT && pin < PWM_COUNT){
		return brd->pwm[pin - PWM_FRONT];
	} 

	return 0; 
}


