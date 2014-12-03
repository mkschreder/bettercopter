#pragma once

#include <math.h>

#define GLM_FORCE_RADIANS

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#include "SensorProvider.hpp"
#include "PID.hpp"

//typedef unsigned short uint16_t;
typedef signed short ivalue; 
typedef unsigned short uivalue; 
/*typedef glm::i16vec4 ivec4;
typedef glm::i16vec3 ivec3;*/

#define MAX_UVALUE(a) (2^(sizeof(a)*8))
#define MAX_IVALUE (MAX_VALUE(ivalue)/2)
#define MIN_IVALUE (-MAX_IVALUE)
#define MAX_UIVALUE (MAX_VALUE(uivalue))

enum {
	PID_STAB_PITCH, 
	PID_STAB_YAW, 
	PID_STAB_ROLL, 
	PID_RATE_PITCH, 
	PID_RATE_YAW, 
	PID_RATE_ROLL,
	PID_COUNT
}; 

/**
 * Flight controller that accepts inputs from RC control and calculates
 * motor thrust.
 */
 
class FlightController {
public:
	FlightController(); 
	/*FlightController(SensorProvider *imu):FlightController(){
		mSensors = imu;
	}*/
	/*
	void setSensorProvider(SensorProvider *sensors) {
		mSensors = sensors;
	}*/
	void updateSensors(
		const glm::vec3 &acc, 
		const glm::vec3 &gyr, 
		const glm::vec3 &mag, 
		float A, float P, float T) {
		mAcc = acc; mGyr = gyr; mMag = mag; 
		mAltitude = A; 
		mPressure = P; 
		mTemperature = T; 
	}
	
	void reset(); 
	
	inline const glm::i16vec4 &getMotorThrust(){return mThrottle;}

	/// the copter is controlled using commands
	//virtual void command(RC_Command cmd, ivalue value);
	virtual void update(
		int16_t thr, 
		int16_t yaw, 
		int16_t pitch, 
		int16_t roll, 
		int16_t mode, 
		timeout_t dt);
protected:

	AC_PID mPID[PID_COUNT]; 
	
	// outputs
	glm::i16vec4 mThrottle;
	glm::vec3 mAcc, mGyr, mMag; 
	float mAltitude, mPressure, mTemperature; 
	
	//value mTargetYaw;
	
	/*float mRCThrottle, mRCPitch, mRCYaw, mRCRoll;

	float mAltErrorInt, mAltPrevError; */

}; 
