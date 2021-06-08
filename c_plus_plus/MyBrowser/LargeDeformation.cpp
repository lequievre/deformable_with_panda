/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include "LargeDeformation.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "CommonInterfaces/CommonDeformableBodyBase.h"
#include "Utils/b3ResourcePath.h"

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

///The LargeDeformation shows the contact between volumetric deformable objects and rigid objects.
static btScalar E = 50;  // Young's Modulus
static btScalar nu = 0.3; // Poisson Ratio
static btScalar damping_alpha = 0.1; // Mass Damping
static btScalar damping_beta = 0.01; // Stiffness Damping

class LargeDeformation : public CommonDeformableBodyBase
{
	btDeformableLinearElasticityForce* m_linearElasticity;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_pandaIndex;

public:
	LargeDeformation(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
        m_linearElasticity = 0;
	}
	virtual ~LargeDeformation()
	{
	}
	void initPhysics();

	void exitPhysics();
	
	virtual bool keyboardCallback(int key, int state)
	{
		b3Printf("-> key = %d, state = %d", key, state);
		return false;
	};

	void resetCamera()
	{
        double cameraDistance = 3;
		double cameraPitch = -45;
		double cameraYaw = 100;
		double cameraTargetPos[3] = {0, 0, 0};
	         
		m_guiHelper->resetCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPos[0], cameraTargetPos[1], cameraTargetPos[2]);
	}
    
    void stepSimulation(float deltaTime)
    {
		
		m_linearElasticity->setPoissonRatio(nu);
		m_linearElasticity->setYoungsModulus(E);
		m_linearElasticity->setDamping(damping_alpha, damping_beta);
		
        float internalTimeStep = 1. / 60.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
};


void LargeDeformation::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	
	int mode = eCONNECT_EXISTING_EXAMPLE_BROWSER;
	m_robotSim.setGuiHelper(m_guiHelper);
	bool connected = m_robotSim.connect(mode);
	m_robotSim.configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, 0);
	m_robotSim.configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0);
	m_robotSim.configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0);

	//			0;//m_robotSim.connect(m_guiHelper);
	b3Printf("robotSim connected = %d", connected);
	
	m_robotSim.setGravity(btVector3(0, 0, -9.8));
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	
	{
				m_robotSim.loadURDF("urdf/plane.urdf");
				
	}
	

    // create volumetric soft body from a file named 'tetra_cylinder_50_cm.vtk' saved into 'data' directory
    {
        btSoftBody* psb = btSoftBodyHelpers::CreateFromVtkFile(getDeformableDynamicsWorld()->getWorldInfo(), "data/tetra_cylinder_50_cm.vtk");

        btTransform psbTransform;
	    psbTransform.setIdentity();
	    // btQuaternion(yaw, pitch, roll)
	    // yaw = Z, pitch = Y, roll = X
	    psbTransform.setRotation(btQuaternion(SIMD_PI / 2,0,0));



        getDeformableDynamicsWorld()->addSoftBody(psb);
        psb->scale(btVector3(1, 1, 1));
        psb->translate(btVector3(0, 0, 0));
        psb->transform(psbTransform);
        psb->getCollisionShape()->setMargin(0.1);
        psb->setTotalMass(0.5);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 0.5;
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
		psb->m_sleepingThreshold = 0;
        
        // btDeformableLinearElasticityForce(btScalar mu, btScalar lambda, btScalar damping_alpha = 0.01, btScalar damping_beta = 0.01)
        double init_mu = 100.0;
        double init_lambda = 100.0;
        double init_damping_alpha = 0.01;
        double init_damping_beta = 0.01;
        
        btDeformableLinearElasticityForce* linearElasticity = new btDeformableLinearElasticityForce(init_mu,init_lambda,init_damping_alpha,init_damping_beta);
		m_linearElasticity = linearElasticity;
        getDeformableDynamicsWorld()->addForce(psb, linearElasticity);
        m_forces.push_back(linearElasticity);
    }
    
    {
		//m_robotSim.loadURDF("urdf/kuka_iiwa/model.urdf");
		b3RobotSimulatorLoadUrdfFileArgs args;
		args.m_startPosition.setValue(-0.5, 0, 0);
		args.m_startOrientation.setEulerZYX(0, 0, 0);
		args.m_useMultiBody = false;
		m_pandaIndex = m_robotSim.loadURDF("urdf/franka_panda/panda.urdf",args);
		int numJoints = m_robotSim.getNumJoints(m_pandaIndex);
		b3Printf("Num joints Panda = %d", numJoints);
	}
    
    
    getDeformableDynamicsWorld()->setImplicit(true);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(true);
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.1;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(20);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-3;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = true;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 100;
    // add a few rigid bodies
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
	
    {
		// https://fr.wikipedia.org/wiki/Coefficient_de_Lam%C3%A9
		// https://fr.wikipedia.org/wiki/Module_de_Young
		// Polyéthylène	0,2 à 0,7 GPa = 200000 à 700000 Pa
		
        SliderParams slider("Young's Modulus", &E);
        slider.m_minVal = 0;
        slider.m_maxVal = 700000;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
		// https://fr.wikipedia.org/wiki/Coefficient_de_Poisson
        SliderParams slider("Poisson Ratio", &nu);
        slider.m_minVal = 0.05;
        slider.m_maxVal = 0.49;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
        SliderParams slider("Mass Damping", &damping_alpha);
        slider.m_minVal = 0;
        slider.m_maxVal = 1;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
        SliderParams slider("Stiffness Damping", &damping_beta);
        slider.m_minVal = 0;
        slider.m_maxVal = 0.1;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
       
}

void LargeDeformation::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
    removePickingConstraint();
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}
    // delete forces
    for (int j = 0; j < m_forces.size(); j++)
    {
        btDeformableLagrangianForce* force = m_forces[j];
        delete force;
    }
    m_forces.clear();
    
	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;
}



class CommonExampleInterface* LargeDeformationCreateFunc(struct CommonExampleOptions& options)
{
	return new LargeDeformation(options.m_guiHelper);
}


