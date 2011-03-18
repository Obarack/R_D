#pragma once
#include "btBulletDynamicsCommon.h"

class Controller
{
public:
	friend class Ragdoll;

	Controller(void);
	~Controller(void);

	// fsm
	enum{
		LEFT_STANCE=0,
		RIGHT_F_STRIKE,
		RIGHT_STANCE,
		LEFT_F_STRIKE,
	};

	double checkFSM();

	// pdCtrl
	void controlTorso(Ragdoll* rd, btScalar deltaTime);

	void controlSwingHip();
	// balance feedback
	void balanceFB();
};

