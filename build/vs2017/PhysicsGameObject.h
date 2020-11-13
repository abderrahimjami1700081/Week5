#pragma once
#include "GameObject.h"

#include "btBulletDynamicsCommon.h"
#include <graphics/mesh.h>
class gef::VertexBuffer;
class PhysicsGameObject :
	public GameObject
{
public:
	PhysicsGameObject();
	~PhysicsGameObject();

	void CreateShapeWithVertices(void* vertices, const UInt32 num_vertices, bool isConvex);
	void CreateBodyWithMass(float mass_);
	void Init();
	void CleanUpPhysics();
	btRigidBody* GetBody() { return body; }
	
private:

	float mass;
	bool convex;
	int tag;

	btRigidBody* body;
	btCollisionShape* shape;



};

