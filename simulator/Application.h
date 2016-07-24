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
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <cstdlib>
#include <glm/glm.hpp>

#include "Socket.h"
#include "Copter.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

class Application : public IEventReceiver{
public:
	Application();
	~Application();
	
	void CreateStartScene();
	void CreateBox(const btVector3 &TPosition, const vector3df &TScale, btScalar TMass, const char *texture = "rust0.jpg");
	void CreateSphere(const btVector3 &TPosition, btScalar TRadius, btScalar TMass);
	void UpdatePhysics(u32 TDeltaTime);
	void UpdateRender(btRigidBody *TObject);
	void updateNetwork(); 
	void ClearObjects();
	int GetRandInt(int TMax) { return rand() % TMax; }
	void run();
	Copter *getActiveQuad(){ return activeQuad; }
	virtual bool OnEvent(const SEvent &TEvent);
	
	// Globals
	bool Done = false;
	btDiscreteDynamicsWorld *World;
	btSequentialImpulseConstraintSolver *Solver;
	btDefaultCollisionConfiguration *CollisionConfiguration;
	btCollisionDispatcher *Dispatcher; 
	btBroadphaseInterface *BroadPhase; 
	IrrlichtDevice *irrDevice;
	IVideoDriver *irrDriver;
	ISceneManager *irrScene;
	IGUIEnvironment *irrGUI;
	//IGUIStaticText* irrStatusText; 
	IFileSystem *irrFile;
	ITimer *irrTimer;
	ILogger *irrLog;
	Copter *activeQuad; 
	list<btRigidBody *> Objects;
	int16_t mRCThrottle, mRCYaw, mRCPitch, mRCRoll; 
	SocketAPM sock; 
	u32 TimeStamp, DeltaTime; 
};

