#include "PhysicsGameObject.h"
#include <graphics/vertex_buffer.h>


PhysicsGameObject::PhysicsGameObject()
{

	Init();

}


PhysicsGameObject::~PhysicsGameObject()
{
}

void PhysicsGameObject::CreateShapeWithVertices(void* vertices, const UInt32 num_vertices, bool isConvex)
{
	gef::Mesh::Vertex* vertexPtr = (gef::Mesh::Vertex*)vertices;


	if (isConvex)
	{
		shape = new btConvexHullShape();
		for (int i = 0; i < num_vertices; i++)
		{
			gef::Mesh::Vertex v = vertexPtr[i];
			btVector3 btv = btVector3(v.px, v.py, v.pz);
			((btConvexHullShape*)shape)->addPoint(btv);
		}
	}

	else
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		for (int i = 0; i < num_vertices; i+=3)
		{
			gef::Mesh::Vertex v1 = vertexPtr[i];
			gef::Mesh::Vertex v2 = vertexPtr[i+1];
			gef::Mesh::Vertex v3 = vertexPtr[i+2];

			btVector3 bv1 = btVector3(v1.px, v1.py, v1.pz);
			btVector3 bv2 = btVector3(v2.px, v2.py, v2.pz);
			btVector3 bv3 = btVector3(v3.px, v3.py, v3.pz);


			mesh->addTriangle(bv1, bv2, bv3);
		}
		shape = new btBvhTriangleMeshShape(mesh, true);
	}



}

void PhysicsGameObject::CreateBodyWithMass(float mass_)
{

	btQuaternion rotation;
	rotation.setEulerZYX(rotationX_, rotationY_, rotationZ_);
	btVector3 position = btVector3(position.x(), position.y(), position.z());

	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(rotation, position));
	btScalar bodyMass = mass_;
	btVector3 bodyInertia;
	shape->calculateLocalInertia(bodyMass, bodyInertia);

	btRigidBody::btRigidBodyConstructionInfo bodyCI = btRigidBody::btRigidBodyConstructionInfo(bodyMass, motionState, shape, bodyInertia);
	bodyCI.m_restitution = 1.0f;
	bodyCI.m_friction = .5f;

	body = new btRigidBody(bodyCI);


	// Not sure what is happening on line 86, need to ask Chris maybe??
	//_body->setUserPointer((__bridge void*)self);

	////9
	//_body->setLinearFactor(btVector3(1, 1, 0));

}

void PhysicsGameObject::Init()
{


	gef::Mesh* meshPtr = (gef::Mesh*)mesh();
	CreateShapeWithVertices(meshPtr->vertex_buffer()->vertex_data(), mesh()->GetNumVertices(), true);
	CreateBodyWithMass(1.f);

}

void PhysicsGameObject::CleanUpPhysics()
{
	if (body)
	{
		delete body->getMotionState();
		delete body;
		body = NULL;
	}

	delete shape;
	shape = NULL;

}






