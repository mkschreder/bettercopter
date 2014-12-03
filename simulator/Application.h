#pragma once

#include <irrlicht.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <cstdlib>
#include <glm/glm.hpp>

#include "CopterEntity.h"

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
	void ClearObjects();
	int GetRandInt(int TMax) { return rand() % TMax; }
	void run();
	CopterEntity *getActiveQuad(){ return activeQuad; }
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
	CopterEntity *activeQuad; 
	list<btRigidBody *> Objects;
	int16_t mRCThrottle, mRCYaw, mRCPitch, mRCRoll; 
	u32 TimeStamp, DeltaTime; 
};

