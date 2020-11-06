#include "GameObject.h"
#include <maths/math_utils.h>



GameObject::GameObject()
{
	Init();
}


GameObject::~GameObject()
{
}

void GameObject::Init()
{
	velocity_ = gef::Vector4(0.0f, 0.0f, 0.0f, 0.0f);
	position_ = gef::Vector4(0.0f, 0.0f, 0.0f, 0.0f);
	rotationX_ = 0.0f;
	rotationY_ = 0.0f;
	rotationZ_ = 0.0f;

	localTransform_.SetIdentity();

	scale_ = { 1.0f, 1.0f, 1.0f };
}

bool GameObject::Update(float frame_time)
{
	position_ += velocity_ * frame_time;
	BuildTransformationMatrix();
	return true;
}



gef::Matrix44 GameObject::BuildRotationX(float RadAngle_)
{
	gef::Matrix44 rotation;
	rotation.SetIdentity();
	rotation.RotationX(gef::DegToRad(RadAngle_));
	return rotation;
}

gef::Matrix44 GameObject::BuildRotationY(float RadAngle_)
{
	gef::Matrix44 rotation;
	rotation.SetIdentity();
	rotation.RotationY(gef::DegToRad(RadAngle_));
	return rotation;
}

gef::Matrix44 GameObject::BuildRotationZ(float RadAngle_)
{
	gef::Matrix44 rotation;
	rotation.SetIdentity();
	rotation.RotationZ(gef::DegToRad(RadAngle_));
	return rotation;
}

gef::Matrix44 GameObject::BuildScale(gef::Vector4 scale)
{
	gef::Matrix44 Scale;
	Scale.SetIdentity();
	Scale.Scale(scale);
	return Scale;

}

gef::Matrix44 GameObject::BuildTranslate(gef::Vector4 tranlate)
{
	gef::Matrix44 Trans;
	Trans.SetIdentity();
	Trans.SetTranslation(tranlate);
	return Trans;
}

void GameObject::BuildTransformationMatrix()
{
	gef::Matrix44 rotation_X,
		rotation_Y,
		rotation_Z,
		Scaleing,
		translate;

	Scaleing = BuildScale(scale_);
	rotation_X = BuildRotationX(rotationX_);
	rotation_Y = BuildRotationY(rotationY_);
	rotation_Z = BuildRotationZ(rotationZ_);
	translate = BuildTranslate(position_);

	localTransform_= Scaleing * rotation_X * rotation_Y * rotation_Z * translate;
}


