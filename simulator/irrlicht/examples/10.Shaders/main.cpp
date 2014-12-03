/** Example 010 Shaders

This tutorial shows how to use shaders for D3D8, D3D9, OpenGL, and Cg with the
engine and how to create new material types with them. It also shows how to
disable the generation of mipmaps at texture loading, and how to use text scene
nodes.

This tutorial does not explain how shaders work. I would recommend to read the
D3D, OpenGL, or Cg documentation, to search a tutorial, or to read a book about
this.

At first, we need to include all headers and do the stuff we always do, like in
nearly all other tutorials:
*/
#include <irrlicht.h>
#include <iostream>
#include "driverChoice.h"

using namespace irr;

#ifdef _MSC_VER
#pragma comment(lib, "Irrlicht.lib")
#endif

/*
Because we want to use some interesting shaders in this tutorials, we need to
set some data for them to make them able to compute nice colors. In this
example, we'll use a simple vertex shader which will calculate the color of the
vertex based on the position of the camera.
For this, the shader needs the following data: The inverted world matrix for
transforming the normal, the clip matrix for transforming the position, the
camera position and the world position of the object for the calculation of the
angle of light, and the color of the light. To be able to tell the shader all
this data every frame, we have to derive a class from the
IShaderConstantSetCallBack interface and override its only method, namely
OnSetConstants(). This method will be called every time the material is set.
The method setVertexShaderConstant() of the IMaterialRendererServices interface
is used to set the data the shader needs. If the user chose to use a High Level
shader language like HLSL instead of Assembler in this example, you have to set
the variable name as parameter instead of the register index.
*/

IrrlichtDevice* device = 0;
bool UseHighLevelShaders = false;
bool UseCgShaders = false;

class MyShaderCallBack : public video::IShaderConstantSetCallBack
{
public:

	virtual void OnSetConstants(video::IMaterialRendererServices* services,
			s32 userData)
	{
		video::IVideoDriver* driver = services->getVideoDriver();

		// set inverted world matrix
		// if we are using highlevel shaders (the user can select this when
		// starting the program), we must set the constants by name.

		core::matrix4 invWorld = driver->getTransform(video::ETS_WORLD);
		invWorld.makeInverse();

		services->setVertexShaderConstant("mInvWorld", invWorld.pointer(), 16);
		
		// set clip matrix

		core::matrix4 worldViewProj;
		worldViewProj = driver->getTransform(video::ETS_PROJECTION);
		worldViewProj *= driver->getTransform(video::ETS_VIEW);
		worldViewProj *= driver->getTransform(video::ETS_WORLD);

		services->setVertexShaderConstant("mWorldViewProj", worldViewProj.pointer(), 16);
		
		core::matrix4 worldView = driver->getTransform(video::ETS_VIEW) * driver->getTransform(video::ETS_WORLD); 
		services->setVertexShaderConstant("mWorldView", worldView.pointer(), 16);
		core::matrix4 view = driver->getTransform(video::ETS_VIEW); 
		core::matrix4 world = driver->getTransform(video::ETS_WORLD); 
		core::matrix4 worldInverse; 
		world.getInverse(worldInverse);
		services->setVertexShaderConstant("mView", view.pointer(), 16);
		
		// set camera position

		core::vector3df pos = device->getSceneManager()->
			getActiveCamera()->getAbsolutePosition(); 
			
		// light position to eye space
		core::vector3df lightPos = pos; //core::vector3df(0, 0, 100); 
		view.transformVect(lightPos);
		
		printf("View: %f %f %f, Light: %f %f %f\r", pos.X, pos.Y, pos.Z, lightPos.X, lightPos.Y, lightPos.Z); 
		services->setVertexShaderConstant("mLightPos", reinterpret_cast<f32*>(&lightPos), 3);
		services->setVertexShaderConstant("mEyePos", reinterpret_cast<f32*>(&pos), 3);
		
		// set light color

		video::SColorf col(0.0f,1.0f,1.0f,0.0f);
		video::SColorf white(1.0f,1.0f,1.0f,1.0f);
		video::SColorf black(0.0f,0.0f,0.0f,1.0f);
		video::SColorf specular(0.0f,1.0f,0.0f,1.0f);
		
		services->setVertexShaderConstant("mLightColor", reinterpret_cast<f32*>(&col), 4);
		
		// set transposed world matrix

		services->setVertexShaderConstant("mWorld", world.pointer(), 16);
		
		core::matrix4 worldTransposed = world.getTransposed();
		core::matrix4 worldInvTransposed = world.getTransposed();
		services->setVertexShaderConstant("mTransWorld", worldTransposed.pointer(), 16);
		services->setVertexShaderConstant("mWorldInv", worldInverse.pointer(), 16);
		services->setVertexShaderConstant("mInvTransWorld", 
			worldInvTransposed.pointer(), 16);

		// set texture, for textures you can use both an int and a float setPixelShaderConstant interfaces (You need it only for an OpenGL driver).
		s32 tex;
		tex = 0; services->setPixelShaderConstant("diffuseTexture", &tex, 1);
		tex = 1; services->setPixelShaderConstant("normalTexture", &tex, 1);
		services->setPixelShaderConstant("fvLowTone", reinterpret_cast<f32*>(&black), 4);
		services->setPixelShaderConstant("fvSpecular", reinterpret_cast<f32*>(&specular), 4);
		services->setPixelShaderConstant("fvHighTone", reinterpret_cast<f32*>(&white), 4);
		f32 power = 2; 
		services->setPixelShaderConstant("fSpecularPower", &power, 1);
		tex = 1; 
		services->setPixelShaderConstant("cube", &tex, 1);
		//services->setPixelShaderConstant("cube", &TextureLayerID, 2);

	}
};

/*
The next few lines start up the engine just like in most other tutorials
before. But in addition, we ask the user if he wants to use high level shaders
in this example, if he selected a driver which is capable of doing so.
*/
int main()
{
	// ask user for driver
	video::E_DRIVER_TYPE driverType=video::EDT_OPENGL;
	
	// ask the user if we should use high level shaders for this example
	if (driverType == video::EDT_DIRECT3D9 ||
		 driverType == video::EDT_OPENGL)
	{
		UseHighLevelShaders = true;
		UseCgShaders = false; 
		
	}

	// create device
	//device = createDevice(driverType, core::dimension2d<u32>(640, 480));
	
	irr::SIrrlichtCreationParameters params;
	params.DriverType=video::EDT_OPENGL;
	params.WindowSize=core::dimension2d<u32>(640, 480);
	params.Bits=32;
	params.Fullscreen=false;
	params.Stencilbuffer=false;
	params.Vsync=false;
	params.AntiAlias=32;

	device = createDeviceEx(params);
	
	if (device == 0)
		return 1; // could not create selected driver.

	video::IVideoDriver* driver = device->getVideoDriver();
	scene::ISceneManager* smgr = device->getSceneManager();
	gui::IGUIEnvironment* gui = device->getGUIEnvironment();

	// Make sure we don't try Cg without support for it
	if (UseCgShaders && !driver->queryFeature(video::EVDF_CG))
	{
		printf("Warning: No Cg support, disabling.\n");
		UseCgShaders=false;
	}

	if (!driver->queryFeature(video::EVDF_PIXEL_SHADER_1_1) &&
		!driver->queryFeature(video::EVDF_ARB_FRAGMENT_PROGRAM_1))
	{
		device->getLogger()->log("WARNING: Pixel shaders disabled "\
			"because of missing driver/hardware support.");
	}

	if (!driver->queryFeature(video::EVDF_VERTEX_SHADER_1_1) &&
		!driver->queryFeature(video::EVDF_ARB_VERTEX_PROGRAM_1))
	{
		device->getLogger()->log("WARNING: Vertex shaders disabled "\
			"because of missing driver/hardware support.");
	}

	video::IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();
	s32 newMaterialType1 = 0;
	s32 newMaterialType2 = 0;

	if (gpu)
	{
		MyShaderCallBack* mc = new MyShaderCallBack();
				
		newMaterialType1 = gpu->addHighLevelShaderMaterialFromFiles(
			"carpaint.vp", "vertexMain", video::EVST_VS_1_1,
			"carpaint.fp", "pixelMain", video::EPST_PS_1_1,
			mc, video::EMT_SOLID, 0, video::EGSL_DEFAULT);

		newMaterialType2 = gpu->addHighLevelShaderMaterialFromFiles(
			"shader.vp", "vertexMain", video::EVST_VS_1_1,
			"shader.fp", "pixelMain", video::EPST_PS_1_1,
			mc, video::EMT_TRANSPARENT_ADD_COLOR, 0 , video::EGSL_DEFAULT);

		mc->drop();
	}

	/*
	Now it's time for testing the materials. We create a test cube and set
	the material we created. In addition, we add a text scene node to the
	cube and a rotation animator to make it look more interesting and
	important.
	*/

	// create test scene node 1, with the new created material type 1
	
	
	//scene::ISceneNode* node = smgr->addCubeSceneNode(50);
	scene::IAnimatedMesh* m = device->getSceneManager()->getMesh( "mesh.obj" );
	irr::scene::IMesh* tangentMesh = smgr->getMeshManipulator()->
                                createMeshWithTangents(m->getMesh(0));
	scene::ISceneNode* node = device->getSceneManager()->addMeshSceneNode(tangentMesh);
	
	node->setPosition(core::vector3df(0,0,0));
	node->setMaterialTexture(0, driver->getTexture("../../media/classic_10.jpg"));
	node->setMaterialTexture(1, driver->getTexture("../../media/classic_10_n.png"));
	node->setMaterialFlag(video::EMF_LIGHTING, true);
	node->setMaterialType((video::E_MATERIAL_TYPE)newMaterialType1);

	printf("Material count: %d\n", node->getMaterialCount()); 
	for(int c = 0; c < node->getMaterialCount(); c++) {
		auto t = node->getMaterial(0).getTexture(0); 
		if(!t)continue; 
		printf("Material %d: %s\n", c, irr::io::path(t->getName()).c_str()); 
	}
	
	smgr->addTextSceneNode(gui->getBuiltInFont(),
			L"PS & VS & EMT_SOLID",
			video::SColor(255,255,255,255),	node);

	scene::ISceneNodeAnimator* anim = smgr->createRotationAnimator(
			core::vector3df(0,0.3f,0));
	//node->addAnimator(anim);
	anim->drop();

	/*
	Same for the second cube, but with the second material we created.
	*/

	// create test scene node 2, with the new created material type 2

	node = device->getSceneManager()->addOctreeSceneNode(m->getMesh(0));
	
	node->setPosition(core::vector3df(0,0,200));
	node->setMaterialTexture(0, driver->getTexture("../../media/wall.bmp"));
	node->setMaterialFlag(video::EMF_LIGHTING, false);
	node->setMaterialFlag(video::EMF_BLEND_OPERATION, true);
	node->setMaterialType((video::E_MATERIAL_TYPE)newMaterialType2);

	smgr->addTextSceneNode(gui->getBuiltInFont(),
			L"PS & VS & EMT_TRANSPARENT",
			video::SColor(255,255,255,255),	node);

	anim = smgr->createRotationAnimator(core::vector3df(0,0.3f,0));
	//node->addAnimator(anim);
	anim->drop();

	/*
	Then we add a third cube without a shader on it, to be able to compare
	the cubes.
	*/

	// add a scene node with no shader

	node = device->getSceneManager()->addOctreeSceneNode(m->getMesh(0));
	
	node->setPosition(core::vector3df(0,0,100));
	//node->setMaterialTexture(0, driver->getTexture("../../media/wall.bmp"));
	//node->setMaterialFlag(video::EMF_LIGHTING, false);
	smgr->addTextSceneNode(gui->getBuiltInFont(), L"NO SHADER",
		video::SColor(255,255,255,255), node);

	/*
	And last, we add a skybox and a user controlled camera to the scene.
	For the skybox textures, we disable mipmap generation, because we don't
	need mipmaps on it.
	*/

	// add a nice skybox

	driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);

	smgr->addSkyBoxSceneNode(
		driver->getTexture("../../media/irrlicht2_up.jpg"),
		driver->getTexture("../../media/irrlicht2_dn.jpg"),
		driver->getTexture("../../media/irrlicht2_lf.jpg"),
		driver->getTexture("../../media/irrlicht2_rt.jpg"),
		driver->getTexture("../../media/irrlicht2_ft.jpg"),
		driver->getTexture("../../media/irrlicht2_bk.jpg"));

	driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);

	// add a camera and disable the mouse cursor

	scene::ICameraSceneNode* cam = smgr->addCameraSceneNodeFPS();
	cam->setPosition(core::vector3df(10,10,10));
	cam->setTarget(core::vector3df(0,0,0));
	device->getCursorControl()->setVisible(false);

	/*
	Now draw everything. That's all.
	*/

	int lastFPS = -1;
	
	smgr->saveScene("output.irr"); 
	
	while(device->run())
		if (device->isWindowActive())
	{
		driver->beginScene(true, true, video::SColor(255,0,0,0));
		smgr->drawAll();
		driver->endScene();

		int fps = driver->getFPS();

		if (lastFPS != fps)
		{
			core::stringw str = L"Irrlicht Engine - Vertex and pixel shader example [";
			str += driver->getName();
			str += "] FPS:";
			str += fps;

			device->setWindowCaption(str.c_str());
			lastFPS = fps;
		}
	}

	device->drop();

	return 0;
}

/*
Compile and run this, and I hope you have fun with your new little shader
writing tool :).
**/
