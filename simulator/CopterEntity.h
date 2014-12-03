#pragma once


#include <irrlicht.h>
#include "sim_kernel.h"

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

//#include <FlightController.hpp>

using namespace glm; 

class Application;

class CopterIMU  {
	friend class CopterEntity; 
public:
	CopterIMU(btDynamicsWorld *world, btRigidBody *body);

	virtual glm::quat getOrientation();
	virtual glm::mat3 getMatrix() {
		return glm::mat3(); 
	}
	virtual glm::vec3 getPosition();

	virtual glm::vec3 getAngularVelocity();
	virtual glm::vec3 getLinearVelocity();

	virtual glm::vec3 getAccelerometer(){return mAcc;}
	virtual glm::vec3 getGyroscope(){return mGyro;}
	virtual glm::vec3 getMagnetometer(){return mMag;}
	/*
	uivalue	readBarometer(){return mPressure;}
	uivalue readTemperature(){return mTemp;}
	uivalue readDistance() { return mDistance; }
	uivalue readAltitude() { return mAltitude; }
	ivec3 readCompass(){return mCompass;}*/
	
	void update(float dt);
protected:
	bool RaycastWorld(const btVector3 &Start, btVector3 &End, btVector3 &Normal);
	glm::vec3 mAcc, mMag, mGyro, mCompass;
	btVector3 mOldVelocity; 
	uint32_t mDistance, mTemp, mPressure, mAltitude; 
private:
	btRigidBody *mBody;
	btDynamicsWorld *World; 
};

class CopterEntity {
public:
	CopterEntity(Application *app);
	//virtual void command(RC_Command cmd, ivalue value) override;
	virtual void update(
		int16_t thr, 
		int16_t yaw, 
		int16_t pitch, 
		int16_t roll, 
		int16_t mode, uint32_t dt);
	virtual void render(irr::video::IVideoDriver *drv);
	
	virtual irr::scene::ISceneNode *getSceneNode(){return mNode;}
	virtual void attachCamera(irr::scene::ICameraSceneNode *cam){
		mCamera = cam;
		//mCamera->setRotation(irr::core::vector3df(45, 0, 0)); 
		mNode->addChild(mCamera); 
	}
	void getSensorData(
		glm::vec3 *acc, 
		glm::vec3 *gyr, 
		glm::vec3 *mag, 
		float *A, float *P, float *T) {
		if(acc) *acc = mSensors->getAccelerometer(); 
		if(gyr) *gyr = mSensors->getGyroscope(); 
		if(mag) *mag = mSensors->getMagnetometer(); 
		if(A) *A = mSensors->getPosition().y; 
		if(P) *P = 0; 
		if(T) *T = 26; 
	}
private:
	CopterIMU *mSensors; 
	btRigidBody *mBody;
	Application *mApp; 
	irr::scene::ISceneNode *mNode;
	irr::scene::ICameraSceneNode *mCamera; 
};
