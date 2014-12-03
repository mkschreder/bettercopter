
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include "Application.h"
#include "kernel.h"

static int paused = 0; 
Application::Application(){
	//int16_t mRCYaw = mRCPitch = mRCRoll = 0; 
	mRCThrottle = 1250; 
	mRCPitch = 1500; 
	mRCYaw = 1500; 
	mRCRoll = 1500; 
	
	irrDevice = createDevice(video::EDT_OPENGL, dimension2d<u32>(640, 480), 32, false, false, false, this);
	irrGUI = irrDevice->getGUIEnvironment();
	irrTimer = irrDevice->getTimer();
	irrScene = irrDevice->getSceneManager();
	irrDriver = irrDevice->getVideoDriver();

	irrDevice->getCursorControl()->setVisible(0);

	// load a map
	irrDevice->getFileSystem()->addFileArchive("map-20kdm2.pk3");
	scene::IAnimatedMesh* mesh = irrScene->getMesh("20kdm2.bsp");
	scene::ISceneNode* node = 0;

	if (mesh)
			node = irrScene->addOctreeSceneNode(mesh->getMesh(0), 0, -1, 1024);
	if (node)
			node->setPosition(core::vector3df(-1300,-144,-1249));
        
	// Initialize bullet
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);

	// Add camera
	ICameraSceneNode *Camera = irrScene->addCameraSceneNodeFPS(0, 100, 0.01);
	//ICameraSceneNode *Camera = irrScene->addCameraSceneNode();
	Camera->setPosition(vector3df(0, 0, 0));
	Camera->setRotation(vector3df(45, -30, 0)); 
	//Camera->setTarget(vector3df(0, -1, 1));
	
	// Preload textures
	irrDriver->getTexture("ice0.jpg");
	irrDriver->getTexture("rust0.jpg");

	// Create text
	//IGUISkin *Skin = irrGUI->getSkin();
	//Skin->setColor(EGDC_BUTTON_TEXT, SColor(255, 255, 255, 255));
	//irrStatusText = irrGUI->addStaticText(L"Hit 1 to create a box\nHit 2 to create a sphere\nHit x to reset", rect<s32>(0, 0, 200, 100), true);

	// Create the initial scene
	irrScene->addLightSceneNode(0, core::vector3df(2, 5, -2), SColorf(4, 4, 4, 1));
	CreateStartScene();

	// Main loop
	TimeStamp = irrTimer->getTime(), DeltaTime = 0;

	CopterEntity *view = new CopterEntity(this);
	activeQuad = view;
	
	//view->attachCamera(Camera); 
}

Application::~Application(){
	ClearObjects();
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;

	irrDevice->drop();
}

void Application::run(){
	DeltaTime = irrTimer->getTime() - TimeStamp;
	TimeStamp = irrTimer->getTime();
	
	if(!paused){
		UpdatePhysics(DeltaTime);
		activeQuad->update(mRCThrottle, mRCYaw, mRCPitch, mRCRoll, 0, DeltaTime);
	}
	
	irrDriver->beginScene(true, true, SColor(255, 20, 0, 0));
	
	irrScene->drawAll();
	
	// debug
	activeQuad->render(irrDriver); 
	
	irrGUI->drawAll();
	irrDriver->endScene();
	irrDevice->run();
}


bool Application::OnEvent(const SEvent &TEvent) {
	static int th = 2220;
	static int ph = 0; 
	
	if(TEvent.EventType == EET_KEY_INPUT_EVENT && !TEvent.KeyInput.PressedDown) {
		switch(TEvent.KeyInput.Key) {
			case KEY_ESCAPE:
				Done = true;
			break;
			case KEY_KEY_Q: 
				paused = ~paused; 
				break; 
			case KEY_KEY_T:
				mRCThrottle += 10;
				break;
			case KEY_KEY_G:
				mRCThrottle -= 10;
				break;
			case KEY_LEFT:
				mRCRoll -= 10;
				break;
			case KEY_RIGHT:
				mRCRoll += 10; 
				break;
			case KEY_UP:
				ph+=1; 
				mRCPitch += 10;
				break;
			case KEY_DOWN:
				ph-=1; 
				mRCPitch -= 10;
				break; 
			case KEY_KEY_F:
				mRCYaw -= 1;
				break; 
			case KEY_KEY_H:
				mRCYaw += 1;
				break; 
			case KEY_KEY_1:
				CreateBox(btVector3(GetRandInt(10) - 5.0f, 7.0f, GetRandInt(10) - 5.0f), vector3df(GetRandInt(3) + 0.5f, GetRandInt(3) + 0.5f, GetRandInt(3) + 0.5f), 500.0f);
			break;
			case KEY_KEY_X:
				CreateStartScene();
			break;
			default:
				return false;
			break;
		}
		
		set_pin(RC_THROTTLE, mRCThrottle); 
		set_pin(RC_ROLL, mRCRoll); 
		set_pin(RC_PITCH, mRCPitch); 
		set_pin(RC_YAW, mRCYaw); 
		set_pin(RC_MODE, 10); 
		
		return true;
	}

	return false;
}

// Runs the physics simulation.
// - TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate.
void Application::UpdatePhysics(u32 TDeltaTime) {

	World->stepSimulation(TDeltaTime * 0.001f, 60);

	// Relay the object's orientation to irrlicht
	for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {

		UpdateRender(*Iterator);
	}	
}

// Creates a base box
void Application::CreateStartScene() {
	ClearObjects();
	CreateBox(btVector3(0.0f, 0.0f, 0.0f), vector3df(10.0f, 0.5f, 10.0f), 0.0f, "ice0.jpg");
}

// Create a box rigid body
void Application::CreateBox(const btVector3 &TPosition, const vector3df &TScale, btScalar TMass, const char *texture) {

	ISceneNode *Node = irrScene->addCubeSceneNode(1.0f);
	Node->setScale(TScale);
	Node->setMaterialFlag(EMF_LIGHTING, 1);
	Node->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, irrDriver->getTexture(texture));

	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(TPosition);

	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btVector3 HalfExtents(TScale.X * 0.5f, TScale.Y * 0.5f, TScale.Z * 0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);

	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(TMass, LocalInertia);

	// Create the rigid body object
	btRigidBody *RigidBody = new btRigidBody(TMass, MotionState, Shape, LocalInertia);

	// Store a pointer to the irrlicht node so we can update it later
	RigidBody->setUserPointer((void *)(Node));

	// Add it to the world
	World->addRigidBody(RigidBody);
	Objects.push_back(RigidBody);
}


// Passes bullet's orientation to irrlicht
void Application::UpdateRender(btRigidBody *TObject) {
	ISceneNode *Node = static_cast<ISceneNode *>(TObject->getUserPointer());

	// Set position
	btVector3 Point = TObject->getCenterOfMassPosition();
	Node->setPosition(vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

	// Set rotation
	vector3df Euler;
	const btQuaternion& TQuat = TObject->getOrientation();
	quaternion q(TQuat.getX(), TQuat.getY(), TQuat.getZ(), TQuat.getW());
	q.toEuler(Euler);
	Euler *= RADTODEG;
	Node->setRotation(Euler);
}

// Removes all objects from the world
void Application::ClearObjects() {

	for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
		btRigidBody *Object = *Iterator;

		// Delete irrlicht node
		ISceneNode *Node = static_cast<ISceneNode *>(Object->getUserPointer());
		Node->remove();

		// Remove the object from the world
		World->removeRigidBody(Object);

		// Free memory
		delete Object->getMotionState();
		delete Object->getCollisionShape();
		delete Object;
	}

	Objects.clear();
}
