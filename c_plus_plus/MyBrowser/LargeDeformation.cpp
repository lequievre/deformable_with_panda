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

#include "Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "Importers/ImportURDFDemo/URDF2Bullet.h"
#include "Importers/ImportURDFDemo/MyMultiBodyCreator.h"



///The LargeDeformation shows the contact between volumetric deformable objects and rigid objects.


// Frite de piscine fait en polyethylene moussé.
// Le Module de Young du Polyéthylène est de 0.2 à 0.7 GPa soit 200000 à 700000 Pa.
// https://www.simulationmateriaux.com/ComportementMecanique/comportement_mecanique_Liste_modules_de_Young.php
// https://fr.wikipedia.org/wiki/Coefficient_de_Lam%C3%A9
// https://fr.wikipedia.org/wiki/Module_de_Young

static btScalar E = 200000;  // Young's Modulus
static btScalar nu = 0.4; // Poisson Ratio
static btScalar damping_alpha = 0; // Mass Damping = 0.1
static btScalar damping_beta = 0; // Stiffness Damping = 0.01



class LargeDeformation : public CommonDeformableBodyBase
{
	btDeformableLinearElasticityForce* m_linearElasticity;
	btDeformableNeoHookeanForce* m_neohookean;
	btMultiBody* m_multiBody;
	

public:
	LargeDeformation(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
        m_linearElasticity = 0;
        m_neohookean = 0;
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
		
		
		/*
		m_neohookean->setPoissonRatio(nu);
		m_neohookean->setYoungsModulus(E);
		*/
		
		
		
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
	//roboticists like Z up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

	createEmptyDynamicsWorld();
	btVector3 gravity(0, 0, 0);
	gravity[upAxis]=-9.8;
	m_dynamicsWorld->setGravity(gravity);
	
	

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
	
	
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.Reset();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	
    {
		
		// The btBoxShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates.
        ///create a ground
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(150.), btScalar(1.0)));
        groundShape->setMargin(0.02);
        m_collisionShapes.push_back(groundShape);
        
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, 0, -1.15));
        //groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
        //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
        btScalar mass(0.);
        
        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);
        
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);
        
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(4);
        
        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
	


    // create volumetric soft body from a file named 'tetra_cylinder_50_cm.vtk' saved into 'data' directory
    {
        btSoftBody* psb = btSoftBodyHelpers::CreateFromVtkFile(getDeformableDynamicsWorld()->getWorldInfo(), "data/tetra_cylinder_50_cm.vtk");

        btTransform psbTransform;
	    psbTransform.setIdentity();
	    // btQuaternion(yaw, pitch, roll)
	    // yaw = Z, pitch = Y, roll = X
	    psbTransform.setRotation(btQuaternion(SIMD_PI / 2,0,0));
		psbTransform.setOrigin(btVector3(1.5,0,0));

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
        
        /* Explode with this parameters :
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        * */
        
		psb->m_sleepingThreshold = 0;
        
        // btDeformableLinearElasticityForce(btScalar mu, btScalar lambda, btScalar damping_alpha = 0.01, btScalar damping_beta = 0.01)
        double init_mu = 100.0;
        double init_lambda = 100.0;
        double init_damping_alpha = 0.01;
        double init_damping_beta = 0.01;
        
        /*
         * Explosion with this :
        btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(20,80,.01);
        getDeformableDynamicsWorld()->addForce(psb, neohookean);
        m_neohookean = neohookean;
        m_forces.push_back(neohookean);
        * */
        
        btDeformableLinearElasticityForce* linearElasticity = new btDeformableLinearElasticityForce(init_mu,init_lambda,init_damping_alpha,init_damping_beta);
		m_linearElasticity = linearElasticity;
        getDeformableDynamicsWorld()->addForce(psb, linearElasticity);
        m_forces.push_back(linearElasticity);
        
        
       /* 
        btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(10,1, true);
        getDeformableDynamicsWorld()->addForce(psb, mass_spring);
        m_forces.push_back(mass_spring);*/
        
        btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        m_forces.push_back(gravity_force);
        
        
       
        
        
         
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
	
	
	
    {
		
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
    

    
    
    BulletURDFImporter u2b(m_guiHelper, 0, 0, 1, 0);
	bool loadOk = u2b.loadURDF("urdf/franka_panda/panda.urdf");  // lwr / kuka.urdf");
	if (loadOk)
	{
		int rootLinkIndex = u2b.getRootLinkIndex();
		b3Printf("urdf root link index = %d\n", rootLinkIndex);
		MyMultiBodyCreator creation(m_guiHelper);
		btTransform identityTrans;
		identityTrans.setIdentity();
		identityTrans.setOrigin(btVector3(0, 0, 0));
		
		ConvertURDF2Bullet(u2b, creation, identityTrans, m_dynamicsWorld, true, u2b.getPathPrefix());
		for (int i = 0; i < u2b.getNumAllocatedCollisionShapes(); i++)
		{
			m_collisionShapes.push_back(u2b.getAllocatedCollisionShape(i));
		}
		m_multiBody = creation.getBulletMultiBody();
		
		if (m_multiBody)
		{
			//kuka without joint control/constraints will gain energy explode soon due to timestep/integrator
			//temporarily set some extreme damping factors until we have some joint control or constraints
			m_multiBody->setAngularDamping(0 * 0.99);
			m_multiBody->setLinearDamping(0 * 0.99);
			b3Printf("Root link name = %s", u2b.getLinkName(u2b.getRootLinkIndex()).c_str());
		}
	}
	else
	{
		b3Printf("Impossible to load Panda URDF !!");
	}
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
       
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


