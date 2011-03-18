
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "../include/RagdollApp.h"

void forcePreTickCB(btDynamicsWorld *world, btScalar timeStep);

void RagdollApp::initPhysics()
{
	// Setup the basic world
	m_time = 0;
	m_CycPerKnee = 2000.f;	// in milliseconds
	m_CycPerHip = 3000.f;	// in milliseconds
	m_fMuscleStrength = 0.5f;

	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;
	m_dynamicsWorld->setInternalTickCallback( forcePreTickCB, this, true );
	m_dynamicsWorld->setGravity( btVector3(0,-0,0) );
	// create surface
	//createSurface();

	//// Setup a big ground box
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-10,0));

	btCollisionObject* fixedGround = new btCollisionObject();
	fixedGround->setCollisionShape(groundShape);
	fixedGround->setWorldTransform(groundTransform);
	m_dynamicsWorld->addCollisionObject(fixedGround);

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	spawnRagdoll(startOffset);
	startOffset.setValue(-1,0.1,0);
	spawnRagdoll(startOffset);

	clientResetScene();		
}

void RagdollApp::spawnRagdoll(const btVector3& startOffset)
{
	Ragdoll* ragDoll = new Ragdoll (m_dynamicsWorld, startOffset);
	m_ragdolls.push_back(ragDoll);
}	

void RagdollApp::createSurface()
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	int i;
	btTransform tr;
	tr.setIdentity();
	const float TRIANGLE_SIZE=1.f;

	//create a triangle-mesh ground
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	const int NUM_VERTS_X = 40;
	const int NUM_VERTS_Y = 5;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;

	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	m_vertices = new btVector3[totalVerts];
	int*	gIndices = new int[totalTriangles*3];
	// set vertex coordinates
	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			float wl = .2f;
			//height of the curved landscape
			float height = sinf(float(i)*wl);//*cosf(float(j)*wl);

			m_vertices[i+j*NUM_VERTS_X].setValue(
				(i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				height,
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
		}
	}

	int index=0;
	// set vertex indices for each triangle mesh
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{	// first triangle
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			// second triangle
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}

	m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(btScalar*) &m_vertices[0].x(),vertStride);

	bool useQuantizedAabbCompression = true;
	groundShape = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression);

	tr.setOrigin(btVector3(0,0,0));		// make it stay at 0,0,0 initially
	// add the ground shape into the collisionshapes
	m_collisionShapes.push_back(groundShape);
	// create ground object
	localCreateRigidBody(0,tr,groundShape);
}

// soo - add
void RagdollApp::setTargetAngVel(btScalar deltaTime)
{
	float ms = deltaTime*1000000.;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	m_time += ms;
	int jnt[2] = {Ragdoll::JOINT_LEFT_KNEE, Ragdoll::JOINT_RIGHT_KNEE};

	for ( int i=0; i<2; i++)
	{
		for( int j=0; j<2; j++ )
		{
			//if( jnt == CONSTRAINT_HINGE )
			btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_ragdolls[i]->GetJoint()[jnt[j]]);
			//else
			//	btConeTwistConstraint* coneC = static_cast<btConeTwistConstraint*>(m_joints[jnt]);

			btScalar curAngle = hingeC->getHingeAngle();

			btScalar fTargetPercent = (int(m_time / 1000) % int(m_CycPerKnee)) / m_CycPerKnee;
			btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
			btScalar fAngleError  = fTargetLimitAngle - curAngle;
			btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
			hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
		}
	}

	//int jntHip[2] = {Ragdoll::JOINT_LEFT_HIP, Ragdoll::JOINT_RIGHT_HIP};
	//for ( int i=0; i<2; i++)
	//{
	//	for( int j=0; j<2; j++ )
	//	{
	//		//if( jnt == CONSTRAINT_HINGE )
	//		//btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_ragdolls[i]->GetJoint()[jntHip[j]]);
	//		//else
	//		btConeTwistConstraint* coneC = static_cast<btConeTwistConstraint*>(m_ragdolls[i]->GetJoint()[jntHip[j]]);

	//		btScalar curAngle = coneC->getTwistAngle();

	//		btScalar fTargetPercent = (int(m_time / 1000) % int(m_CycPerHip)) / m_CycPerHip;
	//		btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
	//		btScalar fTargetLimitAngle = coneC->getSwingSpan1() + fTargetAngle * (coneC->getSwingSpan2() - coneC->getSwingSpan1());
	//		btScalar fAngleError  = fTargetLimitAngle - curAngle;
	//		btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
	//		coneC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
	//	}
	//}
}

void forcePreTickCB(btDynamicsWorld *world, btScalar timeStep)
{
	RagdollApp* rdApp = (RagdollApp*)world->getWorldUserInfo();

	//rdApp->setForces(timeStep/*, (Ragdoll*)rdApp->getRagdolls(0)*/);
	rdApp->setTargetAngVel(timeStep);
}

void RagdollApp::setForces(btScalar deltaTime/*, Ragdoll* rd*/)
{
	btRigidBody *rbr = m_ragdolls[0]->GetBodies(Ragdoll::BP_PELVIS)[0];
	int bdCnt = 11;
	btVector3 relativeForce = -6*bdCnt*m_dynamicsWorld->getGravity();
	relativeForce.setZ(100);
	// apply force on the body frame
	btMatrix3x3& boxRot = rbr->getWorldTransform().getBasis();
	btVector3 correctedForce = boxRot * relativeForce;
	//rbr->applyCentralForce(correctedForce);

	// apply force on the global frame 
	rbr->applyCentralForce(relativeForce);
	//printf("%f\t", correctedForce.length());
}

void RagdollApp::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void RagdollApp::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void RagdollApp::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
			btVector3 startOffset(0,2,0);
			spawnRagdoll(startOffset);
			break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}


}

void	RagdollApp::exitPhysics()
{

	int i;

	for (i=0;i<m_ragdolls.size();i++)
	{
		Ragdoll* doll = m_ragdolls[i];
		delete doll;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them

	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;


}





