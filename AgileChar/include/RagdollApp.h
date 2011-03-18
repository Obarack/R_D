
#ifndef RAGDOLLAPP_H
#define RAGDOLLAPP_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
#include "Ragdoll.h"

class RagdollApp : public GlutDemoApplication
{

	btAlignedObjectArray<class Ragdoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	// surface
	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;

	float m_time;
	float m_CycPerKnee; // in milliseconds
	float m_CycPerHip;	// in milliseconds
	float m_fMuscleStrength;

public:
	void initPhysics();

	void exitPhysics();

	virtual ~RagdollApp()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	void createSurface();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	void setForces(btScalar deltaTime/*, Ragdoll* rd*/);

	void setTargetAngVel(btScalar deltaTime);
	
	Ragdoll* getRagdoll(int rdID) {return m_ragdolls[rdID];}	// soo - add

	static DemoApplication* Create()
	{
		RagdollApp* demo = new RagdollApp();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
