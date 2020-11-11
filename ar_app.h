#ifndef _RENDER_TARGET_APP_H
#define _RENDER_TARGET_APP_H

#include <system/application.h>
#include <graphics/sprite.h>
#include <maths/vector2.h>
#include <vector>
#include <graphics/mesh_instance.h>
#include <platform/vita/graphics/texture_vita.h>
#include "primitive_builder.h"
#include "build\vs2017\GameObject.h"

// Vita AR includes
#include <camera.h>
#include <gxm.h>
#include <motion.h>
#include <libdbg.h>
#include <libsmart.h>



//Bullet Includes
#include "btBulletDynamicsCommon.h"


//Include GameObjectPhysics
#include "build\vs2017\PhysicsGameObject.h"

// FRAMEWORK FORWARD DECLARATIONS
namespace gef
{
	class Platform;
	class SpriteRenderer;
	class Font;
	class Renderer3D;
	class Mesh;
	class RenderTarget;
	class TextureVita;
	class InputManager;
	class Scene;

}


class ARApp : public gef::Application
{
public:
	ARApp(gef::Platform& platform);
	void Init();
	void InitPhysics();
	void CleanUp();
	bool Update(float frame_time);
	void Render();
private:
	void InitFont();
	void CleanUpFont();
	void DrawHUD();
	bool IsCollidingAABB(const gef::MeshInstance& meshInstance1_, const gef::MeshInstance& meshInstance2_);
	void RenderOverlay();
	void SetupLights();

	gef::InputManager* input_manager_;
	void ReadSceneAndAssignFistMesh(const char* filename_, gef::Scene** scene_, gef::Mesh** mesh_);
	gef::Mesh* GetFirstMesh(gef::Scene* scene);

	gef::SpriteRenderer* sprite_renderer_;
	gef::Font* font_;

	float fps_;

	class gef::Renderer3D* renderer_3d_;
	PrimitiveBuilder* primitive_builder_;
	gef::Matrix44 OrthoMatrix;
	gef::Matrix44 ProjectionMatrix;
	float ImageScaleFactor;
	
	gef::Sprite CameraFeedSprite;
	gef::TextureVita TextureCameraMap;

	GameObject Cube_GameObject;
	GameObject SecondCube_GameObject;
	// mesh instance 
	gef::MeshInstance cube;

	//Mesh instance  
	gef::MeshInstance importedModelInstance_;
	class gef::Mesh* importedModelMesh_;

	gef::Scene* model_scene_;


	bool isMarkerFound;
	bool areBoxesColliding;

	PhysicsGameObject PhysicalCube;

	///////Bullet Engine Init
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btDbvtBroadphase* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;







};




#endif // _RENDER_TARGET_APP_H