#include "b3RobotSimulatorClientAPI.h"
#include <stdio.h>

#include "../Utils/b3Clock.h"
#include "Bullet3Common/b3Logging.h"
#include "CommonInterfaces/CommonParameterInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "ExampleBrowser/OpenGLExampleBrowser.h"


static btScalar sGripperClosingTargetVelocity = -0.7f;

int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);
	
	if (!isConnected)
	{
		printf("Cannot connect\n");
		return -1;
	}
	
	//OpenGLGuiHelper my_gui_helper;
	//sim->setGuiHelper(my_gui_helper);
	
	
	sim->resetSimulation(RESET_USE_DEFORMABLE_WORLD);
	
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	
	sim->setTimeOut(10);
	
	//syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();
	
	btScalar fixedTimeStep = 1. / 240.;

	sim->setTimeStep(fixedTimeStep);

	sim->setGravity(btVector3(0, 0, -9.8));
	sim->setAdditionalSearchPath("data");

	sim->loadURDF("plane.urdf");
	
	
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition.setValue(0.0, 0.0, 0.0);
	int pandaIdx = sim->loadURDF("franka_panda/panda.urdf", args);
	
	int numJoints = sim->getNumJoints(pandaIdx);
	b3Printf("numJoints = %d", numJoints);

	for (int i = 0; i < numJoints; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(pandaIdx, i, &jointInfo);
		b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
	}
	
	
	b3RobotSimulatorChangeDynamicsArgs dynamicsArgs;
	int mass = 10;
	dynamicsArgs.m_mass = mass;
	sim->changeDynamics(pandaIdx, -1, dynamicsArgs);
	
	
	
	sim->setRealTimeSimulation(false);
	
	
	b3RobotSimulatorLoadDeformableBodyArgs deformable_args(2, .01, 0.006);
	
	deformable_args.m_springElasticStiffness = 1;
	deformable_args.m_springDampingStiffness = .01;
	deformable_args.m_springBendingStiffness = .1;
	deformable_args.m_frictionCoeff = 10;
	deformable_args.m_useSelfCollision = false;
	deformable_args.m_useFaceContact = true;
	deformable_args.m_useBendingSprings = true;
	deformable_args.m_startPosition.setValue(5, 0, 0);
	deformable_args.m_startOrientation.setValue(1, 0, 0, 1);

	sim->loadDeformableBody("tetra_cylinder_50_cm.vtk",deformable_args);
	
	
	
	bool quit = 0;
	b3KeyboardEventsData keyEvents;
	
	/*GUIHelperInterface *my_gui_helper = sim->getGuiHelper();
	
	if (my_gui_helper == NULL)
		printf("gui helper is NULL !!\n");
	else
		printf("gui helper is not NULL !!\n");
						
						
	
	
	
	SliderParams slider("Closing velocity", &sGripperClosingTargetVelocity);
	
	slider.m_minVal = -1;
	slider.m_maxVal = 1;
	//sim->getGuiHelper()->getParameterInterface()->registerSliderFloatParameter(slider);
	*/
	
	while (sim->canSubmitCommand() && (!quit))
	{
		
		sim->getKeyboardEvents(&keyEvents);
		if (keyEvents.m_numKeyboardEvents)
		{
			for (int i = 0; i < keyEvents.m_numKeyboardEvents; i++)
			{
				b3KeyboardEvent& e = keyEvents.m_keyboardEvents[i];

				if (e.m_keyCode == 'q')
				{
					quit = 1;
				}
				
				if (e.m_keyCode == 'r')
				{
					GUIHelperInterface *my_gui_helper = sim->getGuiHelper();
	
					if (my_gui_helper == NULL)
						printf("gui helper is NULL !!\n");
					else
						printf("gui helper is not NULL !!\n");
				}
			}
		}
		
		sim->stepSimulation();
		
		b3Clock::usleep(1000. * 1000. * fixedTimeStep);
	}
	
	printf("sim->disconnect\n");

	sim->disconnect();

	printf("delete sim\n");
	delete sim;

	printf("exit\n");
	
}

