/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <irrlicht.h>

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

class Copter {
public:
	Copter(Application *app);

	virtual void update(uint32_t dt);
	virtual void render(irr::video::IVideoDriver *drv);
	
	void setServo(unsigned int id, int16_t value); 

	virtual irr::scene::ISceneNode *getSceneNode(){return mNode;}
	virtual void attachCamera(irr::scene::ICameraSceneNode *cam){
		mCamera = cam;
		mNode->addChild(mCamera); 
	}

	glm::quat getRotation(); 
	glm::vec3 getVelocity(); 
	glm::vec3 getPosition(); 
	glm::vec3 getAccel(); 
	glm::vec3 getGyro(); 
private:
	int16_t _servo[8]; 
	btRigidBody *mBody;
	Application *mApp; 
	irr::scene::ISceneNode *mNode;
	irr::scene::ICameraSceneNode *mCamera; 
};
