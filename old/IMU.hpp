#pragma once
/*
#include <avr/io.h>

#include <avr/interrupt.h>
#include <util/delay.h>
*/
#include "Filters.hpp"

#define GLM_FORCE_RADIANS
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <SensorProvider.hpp>

using namespace glm;

/// custom sensor provider

class IMU : public SensorProvider {
public:
	IMU(); 
	// current position and orientation 
	virtual quat getOrientation() override;
	virtual glm::mat3 getMatrix() override; 
	virtual vec3 getPosition() override;

	// rate of change methods
	virtual vec3 getAngularVelocity() override;
	virtual vec3 getLinearVelocity() override;
	
	void setAccelOffset(const glm::vec3 &ofs){
		mAccelOffset = ofs; 
	}
	glm::vec3 getAccelerometer(void) override{
		return mAcc; 
	}
	
	virtual void update(
		int16_t ax, int16_t ay, int16_t az, 
		int16_t gx, int16_t gy, int16_t gz, 
		int16_t mx, int16_t my, int16_t mz, 
		int16_t altitude, int16_t pressure, int16_t temperature, 
		float dt) override;
private:
	vec3 mAcc, mGyro, mMag;
	glm::vec3 mPosition; 
	glm::quat mRotation;
	glm::mat3 mMatrix; 
	glm::vec3 mAccelOffset; 
	
	LPFilter mAccFilter[3];
	HPFilter mGyroFilter[3];
	LPFilter mMagFilter[3];
}; 
