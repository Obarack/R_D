
#include "btBulletDynamicsCommon.h"


#define CONSTRAINT_DEBUG_SIZE 0.2f

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif


class Ragdoll
{
public:
	enum
	{
		BP_PELVIS = 0,
		BP_SPINE,
		BP_HEAD,

		BP_LUL,
		BP_LLL,

		BP_RUL,
		BP_RLL,

		BP_LUA,
		BP_LLA,

		BP_RUA,
		BP_RLA,

		BP_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	enum
	{
		CONSTRAINT_HINGE = 0,
		CONSTRAINT_CONE,

		CONSTRAINT_COUNT
	};

private:
	btDynamicsWorld*	m_ownerWorld;
	btCollisionShape*	m_shapes[11];
	btRigidBody*		m_bodies[11];
	btTypedConstraint*	m_joints[10];
	btScalar			m_weights[11];

	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);


public:
	Ragdoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
		// initialize ragdoll
		initRagdoll(positionOffset);
	}

	virtual	~Ragdoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BP_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);

			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}

	void initRagdoll(const btVector3& positionOffset);
	void initShapes();
	void initWeights();
	void addBodyPart(int consType, const btVector3& trA, const btVector3& trB, 
		const btVector3& oriA, const btVector3& oriB,
		int rbA, int rbB, int jnt, btScalar jLimLow, 
		btScalar jLimHigh, btScalar twistSpan);
	btRigidBody** GetBodies(int body) {return &m_bodies[body];}	// soo - add
	btTypedConstraint** GetJoint() {return &m_joints[0];}	// soo - add

};
