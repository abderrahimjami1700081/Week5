#pragma once

#include <graphics/mesh_instance.h>

class GameObject : public gef::MeshInstance
{
public:
	GameObject();
	~GameObject();

	void Init();
	bool Update(float frame_time);
	gef::Matrix44 BuildRotationX(float RadAngle_);
	gef::Matrix44 BuildRotationY(float RadAngle_);
	gef::Matrix44 BuildRotationZ(float RadAngle_);
	gef::Matrix44 BuildScale(gef::Vector4 scale);
	gef::Matrix44 BuildTranslate(gef::Vector4 Position);


	void BuildTransformationMatrix();
	//virtual void CreateTransMatrix();
	//virtual void Update2_(float frame_time);


	gef::Vector4 velocity_;
	gef::Vector4 position_;
	float rotationX_;
	float rotationY_;
	float rotationZ_;
	gef::Vector4 scale_;
	gef::Matrix44 localTransform_;



};

