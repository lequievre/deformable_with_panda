#include "b3RobotSimulatorClientAPI.h"

#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#ifdef BT_ENABLE_ENET
#include "../SharedMemory/PhysicsClientUDP_C_API.h"
#endif  //PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include "../SharedMemory/PhysicsClientTCP_C_API.h"
#endif  //PHYSICS_TCP

#include "../SharedMemory/PhysicsDirectC_API.h"

#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"

#include "../SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"

#ifdef BT_ENABLE_GRPC
#include "../SharedMemory/PhysicsClientGRPC_C_API.h"
#endif

b3RobotSimulatorClientAPI::b3RobotSimulatorClientAPI()
{
}

b3RobotSimulatorClientAPI::~b3RobotSimulatorClientAPI()
{
}



int b3RobotSimulatorClientAPI::my_loadDeformableBody(const std::string& fileName, const struct b3RobotSimulatorLoadDeformableBodyArgs& args)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int bodyUniqueId;

	b3SharedMemoryCommandHandle command = b3LoadSoftBodyCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	b3LoadSoftBodySetStartPosition(command, args.m_startPosition[0], args.m_startPosition[1], args.m_startPosition[2]);
	b3LoadSoftBodySetStartOrientation(command, args.m_startOrientation[0], args.m_startOrientation[1], args.m_startOrientation[2], args.m_startOrientation[3]);
	b3LoadSoftBodySetScale(command, args.m_scale);
	b3LoadSoftBodySetMass(command, args.m_mass);
	b3LoadSoftBodySetCollisionMargin(command, args.m_collisionMargin);
	if (args.m_NeoHookeanMu > 0)
	{
		b3LoadSoftBodyAddNeoHookeanForce(command, args.m_NeoHookeanMu, args.m_NeoHookeanLambda, args.m_NeoHookeanDamping);
	}
	if (args.m_springElasticStiffness > 0)
	{
		b3LoadSoftBodyAddMassSpringForce(command, args.m_springElasticStiffness, args.m_springDampingStiffness);
	}
	b3LoadSoftBodySetSelfCollision(command, args.m_useSelfCollision);
	b3LoadSoftBodyUseFaceContact(command, args.m_useFaceContact);
	b3LoadSoftBodySetFrictionCoefficient(command, args.m_frictionCoeff);
	b3LoadSoftBodyUseBendingSprings(command, args.m_useBendingSprings, args.m_springBendingStiffness);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_LOAD_SOFT_BODY_COMPLETED)
	{
		b3Warning("Cannot load soft body.");
		bodyUniqueId = -1;
	}
	else
	{
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	
	return bodyUniqueId;
}


b3PhysicsClientHandle& b3RobotSimulatorClientAPI::getPhysicsClientHandle()
{
	return m_data->m_physicsClientHandle;
}



void b3RobotSimulatorClientAPI::renderScene()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	if (m_data->m_guiHelper)
	{
		b3InProcessRenderSceneInternal(m_data->m_physicsClientHandle);
	}
}

void b3RobotSimulatorClientAPI::debugDraw(int debugDrawMode)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	if (m_data->m_guiHelper)
	{
		b3InProcessDebugDrawInternal(m_data->m_physicsClientHandle, debugDrawMode);
	}
}

bool b3RobotSimulatorClientAPI::mouseMoveCallback(float x, float y)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (m_data->m_guiHelper)
	{
		return b3InProcessMouseMoveCallback(m_data->m_physicsClientHandle, x, y) != 0;
	}
	return false;
}
bool b3RobotSimulatorClientAPI::mouseButtonCallback(int button, int state, float x, float y)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (m_data->m_guiHelper)
	{
		return b3InProcessMouseButtonCallback(m_data->m_physicsClientHandle, button, state, x, y) != 0;
	}
	return false;
}

bool b3RobotSimulatorClientAPI::connect(int mode, const std::string& hostName, int portOrKey)
{
	if (m_data->m_physicsClientHandle)
	{
		b3Warning("Already connected, disconnect first.");
		return false;
	}
	b3PhysicsClientHandle sm = 0;

	int udpPort = 1234;
	int tcpPort = 6667;
	int key = SHARED_MEMORY_KEY;

	switch (mode)
	{
		case eCONNECT_EXISTING_EXAMPLE_BROWSER:
		{
			sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(m_data->m_guiHelper);
			break;
		}

		case eCONNECT_GUI:
		{
			int argc = 0;
			char* argv[1] = {0};
#ifdef __APPLE__
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
			break;
		}
		case eCONNECT_GUI_SERVER:
		{
			int argc = 0;
			char* argv[1] = {0};
#ifdef __APPLE__
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
			break;
		}
		case eCONNECT_DIRECT:
		{
			sm = b3ConnectPhysicsDirect();
			break;
		}
		case eCONNECT_SHARED_MEMORY:
		{
			if (portOrKey >= 0)
			{
				key = portOrKey;
			}
			sm = b3ConnectSharedMemory(key);
			break;
		}
		case eCONNECT_UDP:
		{
			if (portOrKey >= 0)
			{
				udpPort = portOrKey;
			}
#ifdef BT_ENABLE_ENET

			sm = b3ConnectPhysicsUDP(hostName.c_str(), udpPort);
#else
			b3Warning("UDP is not enabled in this build");
#endif  //BT_ENABLE_ENET

			break;
		}
		case eCONNECT_TCP:
		{
			if (portOrKey >= 0)
			{
				tcpPort = portOrKey;
			}
#ifdef BT_ENABLE_CLSOCKET

			sm = b3ConnectPhysicsTCP(hostName.c_str(), tcpPort);
#else
			b3Warning("TCP is not enabled in this pybullet build");
#endif  //BT_ENABLE_CLSOCKET
			break;
		}
		case eCONNECT_GRPC:
		{
#ifdef BT_ENABLE_GRPC
			sm = b3ConnectPhysicsGRPC(hostName.c_str(), tcpPort);
#else
			b3Warning("GRPC is not enabled in this pybullet build");
#endif
			break;
		}
		default:
		{
			b3Warning("connectPhysicsServer unexpected argument");
		}
	};

	if (sm)
	{
		m_data->m_physicsClientHandle = sm;
		if (!b3CanSubmitCommand(m_data->m_physicsClientHandle))
		{
			disconnect();
			return false;
		}
		return true;
	}
	return false;
}
