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

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include "Application.h"

static int paused = 0; 
Application::Application():sock(true){
	//int16_t mRCYaw = mRCPitch = mRCRoll = 0; 
	mRCThrottle = 1250; 
	mRCPitch = 1500; 
	mRCYaw = 1500; 
	mRCRoll = 1500; 
	
	mRCThrottle = 2000; 
	mRCPitch = 1000; 
	mRCYaw = 1000; 
	mRCRoll = 1000; 
	
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
	World->setGravity(btVector3(0, -9.82, 0)); 

	// Add camera
	ICameraSceneNode *Camera = irrScene->addCameraSceneNodeFPS(0, 100, 0.01);
	//ICameraSceneNode *Camera = irrScene->addCameraSceneNode();
	Camera->setPosition(vector3df(0, 0, 0));
	Camera->setRotation(vector3df(45, -30, 0)); 
	//Camera->setUpVector(vector3df(0, 0, 1.0)); 
	//Camera->setTarget(vector3df(1, 0, 0));
	
	// Preload textures
	irrDriver->getTexture("ice0.jpg");
	irrDriver->getTexture("rust0.jpg");

	// Create the initial scene
	irrScene->addLightSceneNode(0, core::vector3df(2, 5, -2), SColorf(4, 4, 4, 1));
	irrScene->addLightSceneNode(0, core::vector3df(2, -5, -2), SColorf(4, 4, 4, 1));
	CreateStartScene();

	// Main loop
	TimeStamp = irrTimer->getTime(), DeltaTime = 0;

	Copter *view = new Copter(this);
	activeQuad = view;

	sock.bind("127.0.0.1", 9002); 
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

	printf("%f\n", (float)DeltaTime); 
	if(!paused){
		UpdatePhysics(6);
		activeQuad->update(DeltaTime);
	}
	
	updateNetwork(); 

	irrDriver->beginScene(true, true, SColor(255, 20, 0, 0));
	
	irrScene->drawAll();
	
	// debug
	activeQuad->render(irrDriver); 
	
	irrGUI->drawAll();
	irrDriver->endScene();
	irrDevice->run();
}


bool Application::OnEvent(const SEvent &TEvent) {
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

struct servo_packet {
	int16_t servo[8]; 
}; 

struct state_packet {
	double timestamp; 
	double gyro[3]; 
	double accel[3]; 
	double rcin[8]; 
}; 

void Application::updateNetwork(){
	struct servo_packet pkt; 
	if(sock.recv(&pkt, sizeof(pkt), 0) == sizeof(pkt)){
		printf("servo: %d %d %d %d\n", pkt.servo[0], pkt.servo[1], pkt.servo[2], pkt.servo[3]); 
		for(unsigned c = 0; c < 8; c++){
			activeQuad->setServo(c, pkt.servo[c]); 
		}
	
	struct state_packet state; 
	memset(&state, 0, sizeof(state)); 

	// create a 3d world to ned frame rotation
	glm::quat r1(sin(glm::radians(90.0 / 2)), 0, 0, cos(glm::radians(90.0 / 2))); 
	glm::quat r2(0, sin(glm::radians(90.0 / 2)), 0, cos(glm::radians(90.0 / 2))); 
	glm::quat r2ef = r1 * r2; 
	
	glm::vec3 gyro(0, 0, 0); 

	if(!paused)
		gyro = activeQuad->getGyro(); 
	
	glm::vec3 accel = -activeQuad->getAccel(); 

	state.gyro[0] = -gyro.z; state.gyro[1] = gyro.x; state.gyro[2] = -gyro.y; 
	state.accel[0] = accel.x; state.accel[1] = accel.y; state.accel[2] = accel.z; 
	//state.accel[0] = (mRCPitch - 1000.0) / 100.0; 
	//state.accel[1] = (mRCRoll - 1000.0) / 100.0; 
	//state.accel[2] = (mRCThrottle - 1000.0) / 100.0; 
	state.rcin[0] = (mRCPitch - 1000.0) / 1000.0; 
	state.rcin[1] = (mRCRoll - 1000.0) / 1000.0; 
	state.rcin[2] = (mRCThrottle - 1000.0) / 1000.0; 
	state.rcin[3] = (mRCYaw - 1000.0) / 1000.0; 
	printf("sending: acc(%f %f %f) gyr(%f %f %f) rc(%f %f %f %f)\n", 
		state.accel[0], state.accel[1], state.accel[2],
		gyro.x, gyro.y, gyro.z,
		state.rcin[0], state.rcin[1], state.rcin[2], state.rcin[3]); 
	sock.sendto(&state, sizeof(state), "127.0.0.1", 9003); 

	}
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
