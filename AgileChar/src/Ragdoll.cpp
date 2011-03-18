#include "btBulletDynamicsCommon.h"
#include "../include/Ragdoll.h"

btRigidBody* Ragdoll::localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	// soo - limit movement in 2D
	body->setLinearFactor(btVector3(0,1,1));
	body->setAngularFactor(btVector3(1,0,0));

	m_ownerWorld->addRigidBody(body);

	return body;
}

void Ragdoll::initRagdoll(const btVector3& positionOffset)
{
	// initialize the shapes of the body parts
	initShapes();
	// initialize the weights of the body parts
	initWeights();

	// Setup all the rigid bodies
	btTransform offset; 
	offset.setIdentity();
	//offset.setRotation(btQuaternion())
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
	btScalar rbw = m_weights[BP_PELVIS];
	m_bodies[BP_PELVIS] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
	rbw = m_weights[BP_SPINE];
	m_bodies[BP_SPINE] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_SPINE]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
	rbw = m_weights[BP_HEAD];
	m_bodies[BP_HEAD] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_HEAD]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
	rbw = m_weights[BP_LUL];
	m_bodies[BP_LUL] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_LUL]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
	rbw = m_weights[BP_LLL];
	m_bodies[BP_LLL] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_LLL]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
	rbw = m_weights[BP_RUL];
	m_bodies[BP_RUL] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_RUL]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
	rbw = m_weights[BP_RLL];
	m_bodies[BP_RLL] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_RLL]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	rbw = m_weights[BP_LUA];
	m_bodies[BP_LUA] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_LUA]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	rbw = m_weights[BP_LLA];
	m_bodies[BP_LLA] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_LLA]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	rbw = m_weights[BP_RUA];
	m_bodies[BP_RUA] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_RUA]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	rbw = m_weights[BP_RLA];
	m_bodies[BP_RLA] = localCreateRigidBody(rbw, offset*transform, m_shapes[BP_RLA]);

	// Setup some damping on the m_bodies
	for (int i = 0; i < BP_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05, 0.85);
		m_bodies[i]->setDeactivationTime(DISABLE_DEACTIVATION);
		//m_bodies[i]->setDeactivationTime(0.8);
		//m_bodies[i]->setSleepingThresholds(0, 0);
	}
	// JOINT_PELVIS_SPINE
	addBodyPart(CONSTRAINT_HINGE, btVector3(0,M_PI_2,0), btVector3(0,M_PI_2,0), 
		btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)), 
		BP_PELVIS, BP_SPINE, JOINT_PELVIS_SPINE, 
		-M_PI_4, M_PI_2, 0);
	// JOINT_SPINE_HEAD
	addBodyPart(CONSTRAINT_CONE, btVector3(0,0,M_PI_2), btVector3(0,0,M_PI_2), 
		btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)), 
		BP_SPINE, BP_HEAD, JOINT_SPINE_HEAD, 
		M_PI_4, M_PI_4, M_PI_2);
	// JOINT_LEFT_HIP
	addBodyPart(CONSTRAINT_CONE, btVector3(0,0,-M_PI_4*5), btVector3(0,0,-M_PI_4*5), 
		btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)), 
		BP_PELVIS, BP_LUL, JOINT_LEFT_HIP, 
		M_PI_4, M_PI_4, 0);
	// JOINT_LEFT_KNEE
	addBodyPart(CONSTRAINT_HINGE, btVector3(0,M_PI_2,0), btVector3(0,M_PI_2,0), 
		btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)), 
		BP_LUL, BP_LLL, JOINT_LEFT_KNEE, 
		0, M_PI_2, 0);
	// JOINT_RIGHT_HIP
	addBodyPart(CONSTRAINT_CONE, btVector3(0,0,M_PI_4), btVector3(0,0,M_PI_4), 
		btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)), 
		BP_PELVIS, BP_RUL, JOINT_RIGHT_HIP, 
		M_PI_4, M_PI_4, 0);
	// JOINT_RIGHT_KNEE
	addBodyPart(CONSTRAINT_HINGE, btVector3(0,M_PI_2,0), btVector3(0,M_PI_2,0), 
		btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)), 
		BP_RUL, BP_RLL, JOINT_RIGHT_KNEE, 
		0, M_PI_2, 0);
	// JOINT_LEFT_SHOULDER
	addBodyPart(CONSTRAINT_CONE, btVector3(0,0,M_PI), btVector3(0,0,M_PI_2), 
		btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)), 
		BP_SPINE, BP_LUA, JOINT_LEFT_SHOULDER, 
		M_PI_2, M_PI_2, 0);
	// JOINT_LEFT_ELBOW
	addBodyPart(CONSTRAINT_HINGE, btVector3(0,M_PI_2,0), btVector3(0,M_PI_2,0), 
		btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)), 
		BP_LUA, BP_LLA, JOINT_LEFT_ELBOW, 
		0, M_PI_2, 0);
	// JOINT_RIGHT_SHOULDER
	addBodyPart(CONSTRAINT_CONE, btVector3(0,0,0), btVector3(0,0,M_PI_2), 
		btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)), 
		BP_SPINE, BP_RUA, JOINT_RIGHT_SHOULDER, 
		M_PI_2, M_PI_2, 0);
	// JOINT_RIGHT_ELBOW
	addBodyPart(CONSTRAINT_HINGE, btVector3(0,M_PI_2,0), btVector3(0,M_PI_2,0), 
		btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)),
		btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)), 
		BP_RUA, BP_RLA, JOINT_RIGHT_ELBOW, 
		0, M_PI_2, 0);
}

void Ragdoll::initWeights()
{
	// initialize weights which add up to a total body weight of 75 kg.
	m_weights[BP_PELVIS] = 19;
	m_weights[BP_SPINE]	 = 17.225;
	m_weights[BP_HEAD]	 = 5.325;

	m_weights[BP_LUL] = 7.875;
	m_weights[BP_LLL] = 4.5;

	m_weights[BP_RUL] = 7.875; 
	m_weights[BP_RLL] = 4.5;

	m_weights[BP_LUA] = 2.475;
	m_weights[BP_LLA] = 1.47;

	m_weights[BP_RUA] = 2.475;
	m_weights[BP_RLA] = 1.47;
}

void Ragdoll::initShapes() 
{
	// Setup the geometry
	m_shapes[BP_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
	m_shapes[BP_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
	m_shapes[BP_HEAD] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));
	m_shapes[BP_LUL] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
	m_shapes[BP_LLL] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
	m_shapes[BP_RUL] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
	m_shapes[BP_RLL] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
	m_shapes[BP_LUA] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
	m_shapes[BP_LLA] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
	m_shapes[BP_RUA] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
	m_shapes[BP_RLA] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
}

void Ragdoll::addBodyPart(int consType, const btVector3& trA, const btVector3& trB, 
	const btVector3& oriA, const btVector3& oriB,
	int rbA, int rbB, int jnt, btScalar jLimLow, 
	btScalar jLimHigh, btScalar twistSpan) 
{
	// Now setup the constraints
	btHingeConstraint* hingeC;
	//btConeTwistConstraint* coneC;
	btGeneric6DofConstraint* gen6C;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(trA.getX(),trA.getY(),trA.getZ()); 
	localA.setOrigin(oriA);
	localB.getBasis().setEulerZYX(trB.getX(),trB.getY(),trB.getZ()); 
	localB.setOrigin(oriB);
	if (consType==CONSTRAINT_HINGE)
	{
		hingeC =  new btHingeConstraint(*m_bodies[rbA], *m_bodies[rbB], localA, localB);
		hingeC->setLimit(jLimLow, jLimHigh);
		m_joints[jnt] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	}
	else {
		gen6C = new btGeneric6DofConstraint(*m_bodies[rbA], *m_bodies[rbB], localA, localB, true);
		//coneC = new btConeTwistConstraint(*m_bodies[rbA], *m_bodies[rbB], localA, localB);
		gen6C->setLimit(6, jLimLow, jLimHigh);
		m_joints[jnt] = gen6C;
		gen6C->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	}

	m_ownerWorld->addConstraint(m_joints[jnt], true);
}


