#include "../include/RagdollApp.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char* argv[])
{
        RagdollApp ragdollApp;

        ragdollApp.initPhysics();
		ragdollApp.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Biped",&ragdollApp);
}
