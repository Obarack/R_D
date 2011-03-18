#include "../include/Controller.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"


Controller::Controller(void)
{
}


Controller::~Controller(void)
{
}


double Controller::checkFSM()
{
	// check in which state the character is in
	// if necessary make transition, and return required transition time
	return 0;
}

void controlTorso(Ragdoll* rd, btScalar deltaTime)
{
	float ms = deltaTime*1000000.;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	for( int i=0; i<4; i++ )
	{
		//if( jnt == CONSTRAINT_HINGE )
	//	btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_ragdolls[0]->GetJoint()[Ragdoll::JOINT_LEFT_KNEE]);
		//else
		//	btConeTwistConstraint* coneC = static_cast<btConeTwistConstraint*>(m_joints[jnt]);

//		btScalar curAngle = hingeC->getHingeAngle();

		//btScalar fTargetPercent = (int(m_time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
		//btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
		//btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
		//btScalar fAngleError  = fTargetLimitAngle - curAngle;
		//btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
		//hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
	}

}

void Controller::controlSwingHip()
{

}

void Controller::balanceFB()
{

}
