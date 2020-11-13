#include "ar_app.h"
#include <system/platform.h>
#include <graphics/sprite_renderer.h>
#include <graphics/texture.h>
#include <graphics/mesh.h>
#include <graphics/primitive.h>
#include <assets/png_loader.h>
#include <graphics/image_data.h>
#include <graphics/font.h>
#include <input/touch_input_manager.h>
#include <maths/vector2.h>
#include <input/sony_controller_input_manager.h>
#include <input/input_manager.h>
#include <maths/math_utils.h>
#include <graphics/renderer_3d.h>
#include <graphics/render_target.h>
#include <graphics/scene.h>

#include <sony_sample_framework.h>
#include <sony_tracking.h>


ARApp::ARApp(gef::Platform& platform) :
	Application(platform),
	input_manager_(NULL),
	sprite_renderer_(NULL),
	font_(NULL),
	renderer_3d_(NULL),
	primitive_builder_(NULL)
{
}

void ARApp::Init()
{
	input_manager_ = gef::InputManager::Create(platform_);
	sprite_renderer_ = gef::SpriteRenderer::Create(platform_);
	renderer_3d_ = gef::Renderer3D::Create(platform_);
	primitive_builder_ = new PrimitiveBuilder(platform_);

	// Setup orthographic matrix 
	OrthoMatrix = platform_.OrthographicFrustum(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f);

	ImageScaleFactor = (960.f / 544.f) / (640.f / 480.f);
	CameraFeedSprite.set_position(gef::Vector4(0.0f, 0.0f, 1.0f));
	CameraFeedSprite.set_width(2.f);
	CameraFeedSprite.set_height(2 * ImageScaleFactor);


	// Setup projection  matrix 
	gef::Matrix44 ScaleMatrix;
	ScaleMatrix.Scale(gef::Vector4(1.0f, ImageScaleFactor, 1.0f));
	ProjectionMatrix = platform_.PerspectiveProjectionFov(SCE_SMART_IMAGE_FOV, SCE_SMART_IMAGE_WIDTH/ (float)SCE_SMART_IMAGE_HEIGHT, .1f, 100.f);
	
	ProjectionMatrix = ProjectionMatrix * ScaleMatrix;

	InitFont();
	SetupLights();
	

	// initialise sony framework
	sampleInitialize();
	smartInitialize();

	// reset marker tracking
	AppData* dat = sampleUpdateBegin();
	smartTrackingReset();
	sampleUpdateEnd(dat);



	//Create Cube
	//cube.set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(0.0295f, 0.0295f, 0.0295f), gef::Vector4(0.0f, 0.0f, 0.0f)));
	cube.set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(.015f, .015f, .015f), gef::Vector4(0.0f, 0.0f, 0.0f)));

	//Setup model mesh
	ReadSceneAndAssignFistMesh("ball1.scn", &model_scene_, &importedModelMesh_);
	importedModelInstance_.set_mesh(importedModelMesh_);
	Cube_GameObject.set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(.015f, .015f, .015f), gef::Vector4(0.0f, 0.0f, 0.0f)));
	SecondCube_GameObject.set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(.015f, .015f, .015f), gef::Vector4(0.0f, 0.0f, 0.0f)));
	//Cube_GameObject.position_ = gef::Vector4(0.0f, 0.0f, -0.5f);
	//Cube_GameObject.scale_ = gef::Vector4(0.1f, 0.1f, 0.1f);
	//Cube_GameObject.velocity_ = gef::Vector4(0.0f, 0.0f, -0.2f);
	Cube_GameObject.position_ = gef::Vector4(.3f, 0.0f, 0.0f);
	

	// Bullet setup
	///-----initialization_start-----
	InitPhysics();

	//Initiate PhisicalCube
	PhysicalCube.set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(.015f, .015f, .015f), gef::Vector4(0.0f, 0.0f, 0.0f)));
	dynamicsWorld->addRigidBody(PhysicalCube.GetBody());


	areBoxesColliding = false;
}

void ARApp::InitPhysics()
{

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, 0.0f, -0.7f));
	///-----initialization_end-----
}

void ARApp::CleanUp()
{
	delete primitive_builder_;
	primitive_builder_ = NULL;

	smartRelease();
	sampleRelease();

	CleanUpFont();
	delete sprite_renderer_;
	sprite_renderer_ = NULL;

	delete renderer_3d_;
	renderer_3d_ = NULL;

	delete input_manager_;
	input_manager_ = NULL;


	delete dynamicsWorld;
	dynamicsWorld = NULL;

	delete solver;
	solver = NULL;

	delete collisionConfiguration;
	collisionConfiguration = NULL;

	delete dispatcher;
	dispatcher = NULL;

	delete overlappingPairCache;
	overlappingPairCache = NULL;


}

bool ARApp::Update(float frame_time)
{
	fps_ = 1.0f / frame_time;

	AppData* dat = sampleUpdateBegin();
	Cube_GameObject.Update(frame_time);
	SecondCube_GameObject.Update(frame_time);

	// use the tracking library to try and find markers
	smartUpdate(dat->currentImage);
	//for (int i = 0; i < 6; i++)
	//{

	//	if (sampleIsMarkerFound(i))
	//	{
	//		if (i == 0)
	//		{


	//			gef::Matrix44 markerTransform;
	//			sampleGetTransform(i, &markerTransform);
	//			gef::Matrix44 /*ScaleMatrix, ScaleMatrix2,*/ translationMatrix;
	//			translationMatrix.SetIdentity();
	//			//ScaleMatrix2.SetIdentity();
	//			//ScaleMatrix.SetIdentity();
	//			//ScaleMatrix.Scale(gef::Vector4(0.001f, 0.001f, 0.001f));
	//			//ScaleMatrix.SetIdentity();
	//			//ScaleMatrix = ScaleMatrix * markerTransform;
	//			//importedModelInstance_.set_transform(ScaleMatrix);
	//			//ScaleMatrix2.Scale(gef::Vector4(0.1f, 0.1f, 0.1f));
	//			//translationMatrix.SetTranslation(gef::Vector4(0.0f, 0.0f, 0.2f));
	//			translationMatrix.Scale(gef::Vector4(2.0f, 2.0f, 2.0f));
	//			translationMatrix =/* ScaleMatrix2 **/ translationMatrix * markerTransform;
	//			cube.set_transform(translationMatrix);
	//			//importedModelMesh_->aabb();
	//			//importedModelInstance_.set_transform(ScaleMatrix);


	//			//Multiply cube_GameObject local transform by markerTransform
	//			Cube_GameObject.localTransform_ = Cube_GameObject.localTransform_ * markerTransform;
	//			//Move it along the X
	//		
	//			Cube_GameObject.set_transform(Cube_GameObject.localTransform_);
	//		}

	//		if (i == 3)
	//		{
	//			gef::Matrix44 markerTransform;
	//			sampleGetTransform(i, &markerTransform);

	//			SecondCube_GameObject.localTransform_ = SecondCube_GameObject.localTransform_ * markerTransform;
	//			SecondCube_GameObject.set_transform(SecondCube_GameObject.localTransform_);
	//		}

	//		// //read input devices
	//		//if (input_manager_)
	//		//{
	//		//	input_manager_->Update();
	//		//	// controller input
	//		//	gef::SonyControllerInputManager* controller_manager = input_manager_->controller_input();
	//		//	if (controller_manager)
	//		//	{
	//		//		const gef::SonyController* controller = controller_manager->GetController(0);
	//		//		if (controller)
	//		//		{
	//		//			float left_x_axis = controller->left_stick_x_axis();
	//		//			float left_y_axis = controller->left_stick_y_axis();
	//		//			if (left_x_axis < 0)
	//		//			{
	//		//				Cube_GameObject.position_.set_x(Cube_GameObject.position_.x() - .01f);
	//		//			}
	//		//			if (left_y_axis < 0)
	//		//			{
	//		//				Cube_GameObject.position_.set_y(Cube_GameObject.position_.y() + .01f);
	//		//			}
	//		//			if (left_x_axis > 0)
	//		//			{
	//		//				Cube_GameObject.position_.set_x(Cube_GameObject.position_.x() + .01f);
	//		//			}
	//		//			if (left_y_axis > 0)
	//		//			{
	//		//				Cube_GameObject.position_.set_y(Cube_GameObject.position_.y() - .01f);
	//		//			}
	//		//		}
	//		//	}
	//		//}	

	//	}


	//}
	gef::Matrix44 marker2Transform;		 // Marker 2 transform
	gef::Matrix44 markerTransform;

	if (sampleIsMarkerFound(0))
	{
		sampleGetTransform(0, &markerTransform);		// Marker 1 transform
		gef::Matrix44 /*ScaleMatrix, ScaleMatrix2,*/ translationMatrix;
		translationMatrix.SetIdentity();
		translationMatrix.SetTranslation(gef::Vector4(0.0f, 0.0f, 0.2f));
		translationMatrix.Scale(gef::Vector4(2.0f, 2.0f, 2.0f));
		translationMatrix =/* ScaleMatrix2 **/ translationMatrix * markerTransform;
		cube.set_transform(translationMatrix);
		//Multiply cube_GameObject local transform by markerTransform
		Cube_GameObject.localTransform_ = Cube_GameObject.localTransform_ * markerTransform;
		//Move it along the X

		Cube_GameObject.set_transform(Cube_GameObject.localTransform_);
		if (sampleIsMarkerFound(3))
		{
			sampleGetTransform(3, &marker2Transform);
			gef::Matrix44 InverseMarker1Transform;
			InverseMarker1Transform.AffineInverse(markerTransform);
			gef::Matrix44 LocalMarker2Transform = marker2Transform * InverseMarker1Transform;
			SecondCube_GameObject.localTransform_ = SecondCube_GameObject.localTransform_ * LocalMarker2Transform;

			SecondCube_GameObject.set_transform(SecondCube_GameObject.localTransform_ * markerTransform);

			SecondCube_GameObject.position_.Lerp(SecondCube_GameObject.localTransform_.GetTranslation(), cube.transform().GetTranslation(), frame_time);
			SecondCube_GameObject.Update(frame_time);
			       
			
		}



	}

	dynamicsWorld->stepSimulation(frame_time);


	sampleUpdateEnd(dat);

	// Check for collision
	if (IsCollidingAABB(cube, Cube_GameObject))
	{
		areBoxesColliding = true;
	}
	else
	{
		areBoxesColliding = false;
	}


	return true;
}

void ARApp::Render()
{
	AppData* dat = sampleRenderBegin();
	//
	// Render the camera feed
	//
	



	// REMEMBER AND SET THE PROJECTION MATRIX HERE

	// DRAW CAMERA FEED SPRITE HERE
	if (dat->currentImage)
	{
		//Set the texture of the textureVita object 
		TextureCameraMap.set_texture(dat->currentImage->tex_yuv);
		CameraFeedSprite.set_texture(&TextureCameraMap);
		sprite_renderer_->set_projection_matrix(OrthoMatrix);
		sprite_renderer_->Begin(true);

		// Draw the sprite
		sprite_renderer_->DrawSprite(CameraFeedSprite);

		sprite_renderer_->End();

	}




	//
	// Render 3D scene
	//

	// SET VIEW AND PROJECTION MATRIX HERE
	gef::Matrix44 view_matrix;
	view_matrix.SetIdentity();
	renderer_3d_->set_projection_matrix(ProjectionMatrix);
	renderer_3d_->set_view_matrix(view_matrix);

	// Begin rendering 3D meshes, don't clear the frame buffer
	renderer_3d_->Begin(false);

	// DRAW 3D MESHES HERE
	//renderer_3d_->DrawMesh(importedModelInstance_);
	renderer_3d_->DrawMesh(cube);

	renderer_3d_->DrawMesh(Cube_GameObject);
	renderer_3d_->DrawMesh(SecondCube_GameObject);
	renderer_3d_->End();

	RenderOverlay();

	sampleRenderEnd();
}


void ARApp::RenderOverlay()
{
	//
	// render 2d hud on top
	//
	gef::Matrix44 proj_matrix2d;

	proj_matrix2d = platform_.OrthographicFrustum(0.0f, platform_.width(), 0.0f, platform_.height(), -1.0f, 1.0f);
	sprite_renderer_->set_projection_matrix(proj_matrix2d);
	sprite_renderer_->Begin(false);
	DrawHUD();
	sprite_renderer_->End();
}


void ARApp::InitFont()
{
	font_ = new gef::Font(platform_);
	font_->Load("comic_sans");
}

void ARApp::CleanUpFont()
{
	delete font_;
	font_ = NULL;
}

void ARApp::DrawHUD()
{
	if(font_)
	{
		font_->RenderText(sprite_renderer_, gef::Vector4(850.0f, 510.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "FPS: %.1f", fps_);
		if (areBoxesColliding)
		{
			font_->RenderText(sprite_renderer_, gef::Vector4(700.0f, 410.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "The boxes are colliding");
		}

		//font_->RenderText(sprite_renderer_, gef::Vector4(750.0f, 310.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "CubePos: %.1f", PhysicalCube.GetBody()->getWorldTransform().getOrigin().getY());

	}
}

void ARApp::SetupLights()
{
	gef::PointLight default_point_light;
	default_point_light.set_colour(gef::Colour(0.7f, 0.7f, 1.0f, 1.0f));
	default_point_light.set_position(gef::Vector4(-300.0f, -500.0f, 100.0f));

	gef::Default3DShaderData& default_shader_data = renderer_3d_->default_shader_data();
	default_shader_data.set_ambient_light_colour(gef::Colour(0.5f, 0.5f, 0.5f, 1.0f));
	default_shader_data.AddPointLight(default_point_light);
}


void ARApp::ReadSceneAndAssignFistMesh(const char* filename_, gef::Scene** scene_, gef::Mesh** mesh_)
{
	// Initialize scene  --  Remember to unload assets 
	gef::Scene* scn = new gef::Scene();
	scn->ReadSceneFromFile(platform_, filename_);

	scn->CreateMaterials(platform_);

	// if there is mesh data in the scene, create a mesh to draw from the first mesh
	*mesh_ = GetFirstMesh(scn);
	*scene_ = scn;
}


gef::Mesh* ARApp::GetFirstMesh(gef::Scene* scene)
{
	gef::Mesh* mesh = NULL;

	if (scene)
	{
		// now check to see if there is any mesh data in the file, if so lets create a mesh from it
		if (scene->mesh_data.size() > 0)
			mesh = scene->CreateMesh(platform_, scene->mesh_data.front());
	}

	return mesh;
}


bool ARApp::IsCollidingAABB(const gef::MeshInstance& meshInstance1_, const gef::MeshInstance& meshInstance2_)
{
	gef::Aabb Aabb1_ = meshInstance1_.mesh()->aabb();
	gef::Aabb Aabb2_ = meshInstance2_.mesh()->aabb();

	gef::Aabb Aabb1_t_ = Aabb1_.Transform(meshInstance1_.transform());
	gef::Aabb Aabb2_t_ = Aabb2_.Transform(meshInstance2_.transform());


	if (Aabb1_t_.max_vtx().x() > Aabb2_t_.min_vtx().x() && Aabb1_t_.min_vtx().x() < Aabb2_t_.max_vtx().x() &&
		Aabb1_t_.max_vtx().y() > Aabb2_t_.min_vtx().y() && Aabb1_t_.min_vtx().y() < Aabb2_t_.max_vtx().y() &&
		Aabb1_t_.max_vtx().z() > Aabb2_t_.min_vtx().z() && Aabb1_t_.min_vtx().z() < Aabb2_t_.max_vtx().z()
		)
	{
		return true;
	}

	return false;

}

