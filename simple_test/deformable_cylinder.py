#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

"""

import pybullet as p
import pybullet_data
import os, inspect
from time import sleep

debug_print_vertices = False

def print_mesh_data_vertices(objectId):
  data = p.getMeshData(objectId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
  # data[0] = amount of vertices
  #print("data[0] = ", data[0])
  # data[1] = list of tuples (each tuple contain 3D pos of a vertice)
  #print("data[1] = ", data[1])  
  for i in range(data[0]):
    pos = data[1][i]
    uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
    

# Get current file name (deformable_cylinder.py)
current_frame = inspect.currentframe()
print("current_frame = ", current_frame)

current_file_name = inspect.getfile(current_frame)
print("current_file_name = ", current_file_name)

# Get absolute path of the file deformable_cylinder.py (directory name + file name)
absolute_path_of_current_file = os.path.abspath(current_file_name)
print("absolute_path_of_current_file = ", absolute_path_of_current_file)

# Get directory name of file deformable_cylinder.py
directory_name = os.path.dirname(absolute_path_of_current_file)
print("directory_name = ", directory_name)

# Insert the directory name to system path variable (in position 0 = the first one)
os.sys.path.insert(0, directory_name)

physics_client_id = p.connect(p.GUI)

#p.setTimeStep(timeStep=1./240, physicsClientId=physics_client_id)
p.setTimeStep(0.001, physicsClientId = physics_client_id)

pybullet_data_path = pybullet_data.getDataPath()
print("pybullet_data_path = ", pybullet_data_path)

# Add a search data path
p.setAdditionalSearchPath(pybullet_data_path)
# pybullet data path -> par exemple : /home/laurent/Projects/rl_melodie/ve_panda/lib/python3.8/site-packages/pybullet_data

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -9.81)

p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=physics_client_id)

planId = p.loadURDF("plane.urdf", [0.0,0.0,-0.1])

base_orientation_cylinder = p.getQuaternionFromEuler((0.0, 1.57, 0.0))

base_position_cylinder = [1.775,0.0,0.0]

flags = p.URDF_USE_SELF_COLLISION

# cylinder length = 1.55

#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 100, collisionMargin = 0.005, useMassSpring=0, useBendingSprings=0, useNeoHookean = 1, NeoHookeanMu = 83200, NeoHookeanLambda = 83200, NeoHookeanDamping = 1000, springElasticStiffness=0.0, springDampingStiffness=0.0, springBendingStiffness=0.0, springDampingAllDirections=0, frictionCoeff=.5, useFaceContact=0, useSelfCollision=0, repulsionStiffness=0.0)

#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 5.0, collisionMargin = 0.01, useNeoHookean = 1, NeoHookeanMu = 83200, NeoHookeanLambda = 83200, NeoHookeanDamping = 1000, frictionCoeff=.5)


#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 2.0, collisionMargin = 0.01, useNeoHookean = 1, NeoHookeanMu = 6000, NeoHookeanLambda = 6000, NeoHookeanDamping = 200, frictionCoeff=50, useSelfCollision = 1, repulsionStiffness = 800)

#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 4, useNeoHookean = 1, NeoHookeanMu = 6000, NeoHookeanLambda = 6000, NeoHookeanDamping = 0.1, useSelfCollision = 1, frictionCoeff = 0.50, collisionMargin = 0.001)


# after reading code c++
# m_E = 200000
# m_nu = 0.4

# m_mu = m_E * 0.5 / (1 + m_nu);  ou 0.07
# m_lambda = m_E * m_nu / ((1 + m_nu) * (1 - 2 * m_nu)); ou 0.28

# m_mu = 	200000 * 0.5 / (1 + 0.4) = 100000 / 1.4 = 71428,571428571
# m_lambda = 200000 * 0.4 / ((1 + 0.4) * (1 - 2 * 0.4)) = 80000 / (1.4 * 0.2) = 80000 / 0.28 = 285714,285714286

#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 0.2, useNeoHookean = 1, NeoHookeanMu = 71428, NeoHookeanLambda = 285714, NeoHookeanDamping = 0.01, useSelfCollision = 1, frictionCoeff = 0.50, collisionMargin = 0.001)
#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_5_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 3, useNeoHookean = 1, NeoHookeanMu = 71428, NeoHookeanLambda = 285714, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)

cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_2_5_mm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, mass = 0.02, useNeoHookean = 1, NeoHookeanMu = 96.2, NeoHookeanLambda = 144.2, NeoHookeanDamping =0.01, useSelfCollision = 1, collisionMargin = 0.001, frictionCoeff = 0.5)


#cylinderId = p.loadURDF("deformable_object/frite/frite.urdf", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder)


# Coeff de lamé : https://fr.wikipedia.org/wiki/Coefficient_de_Lam%C3%A9
# Coeff de poisson : https://fr.wikipedia.org/wiki/Coefficient_de_Poisson
# module de young : https://fr.wikipedia.org/wiki/Module_de_Young
# min, max de Neo : https://github.com/bulletphysics/bullet3/issues/3136
# code bullet min, max : https://github.com/bulletphysics/bullet3/blob/master/examples/DeformableDemo/VolumetricDeformable.cpp#L246
# http://www.roto30.fr/polyethylene-tableau-des-caracteristiques/



texUid = p.loadTexture("deformable_object/texture/texture_frite.png")
p.changeVisualShape(cylinderId, -1, textureUniqueId=texUid)

#objectUid = p.loadURDF("random_urdfs/000/000.urdf", globalScaling = 2.0, basePosition=[0.0,0,0.0])

#bunnyId = p.loadSoftBody("deformable_object/torus/torus_textured.obj", simFileName="deformable_object/torus/torus.vtk", mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)


if (debug_print_vertices):
    print_mesh_data_vertices(cylinderId)

#boxId_Left = p.loadURDF("cube.urdf", basePosition = [0.833,0,0.2], globalScaling = 0.05, useMaximalCoordinates = True)
#boxId_Right = p.loadURDF("cube.urdf", basePosition = [-0.775,0,0.2], globalScaling = 0.05, useMaximalCoordinates = True)


basePosition_box_left = [1.3,0.0,0.5]
baseOrientation_box_left = [0.0, 0.0, 0.0, 1.0]

basePosition_box_right = [-1.3,0.0,0.5]
baseOrientation_box_right = [0.0, 0.0, 0.0, 1.0]

#boxId_Left = p.loadURDF("cube.urdf", basePosition = basePosition_box_left, baseOrientation= baseOrientation_box_left, globalScaling = 1.0, useMaximalCoordinates = True, useFixedBase=False)

#boxId_Right = p.loadURDF("cube.urdf", basePosition = basePosition_box_right, baseOrientation= baseOrientation_box_right, globalScaling = 1.0, useMaximalCoordinates = True, useFixedBase=False)



cube_prismatic_left_Id = p.loadURDF("deformable_object/CubePrismatic/cube_prismatic_left.urdf", [0.0,0.0,0.0],flags=p.URDF_USE_SELF_COLLISION)

cube_prismatic_right_Id = p.loadURDF("deformable_object/CubePrismatic/cube_prismatic_right.urdf", [0.0,0.0,0.0],flags=p.URDF_USE_SELF_COLLISION)

p.resetBasePositionAndOrientation(bodyUniqueId=cube_prismatic_right_Id, posObj=[5.0,0.0,0.0], ornObj=[0,0,0,1], physicsClientId=physics_client_id)

"""
p.setJointMotorControl2(bodyUniqueId=cube_prismatic_left_Id, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=0.04,
                                    physicsClientId=physics_client_id)
                                    
p.setJointMotorControl2(bodyUniqueId=cube_prismatic_left_Id, jointIndex=3, controlMode=p.POSITION_CONTROL, targetPosition=0.04,
                                    physicsClientId=physics_client_id)
 """                                   
sleep(1.0)

#p.createSoftBodyAnchor(cylinderId ,5,boxId_Left,-1)
#p.createSoftBodyAnchor(cylinderId ,0,boxId_Right,-1)
#p.addUserDebugText("*", [0.777,0.0,-0.1], textColorRGB=[0,0,0])
#p.createSoftBodyAnchor(cylinderId ,0,boxId_Right,-1)

slider_cube_prismatic_left = p.addUserDebugParameter("left prismatic", 0.0, 2.0, 0) # add a slider for that joint with the limits
slider_cube_prismatic_right = p.addUserDebugParameter("right prismatic", 0.0, 2.0, 0) # add a slider for that joint with the limits
    
slider_cube_revolute_left = p.addUserDebugParameter("left revolute", -2.9, 2.9, 0)

slider_cube_gripper_left = p.addUserDebugParameter("left gripper", 0.0, 0.04, 0.04)

slider_cube_revolute_right = p.addUserDebugParameter("right revolute", -2.9, 2.9, 0)

slider_cube_gripper_right = p.addUserDebugParameter("right gripper", 0.0, 0.04, 0.04)

p.setRealTimeSimulation(0)

while True:

    # look if there is keyboard event and put it in a dictionary named keys
    keys = p.getKeyboardEvents()
    
    # 'Enter' event = 65309, if so .. break the loop
    if 65309 in keys:
      break
    """  
    if 112 in keys: # Grasp the object by using force (with letter 'p' = 112)
      basePosition_box_left[0]-=0.1
      p.resetBasePositionAndOrientation(bodyUniqueId=boxId_Left, posObj=basePosition_box_left, ornObj= baseOrientation_box_left, physicsClientId=physics_client_id)
	   
     """
     
    slider_value_prismatic_left = p.readUserDebugParameter(slider_cube_prismatic_left)
    slider_value_prismatic_right = p.readUserDebugParameter(slider_cube_prismatic_right)
    
    slider_value_revolute_left = p.readUserDebugParameter(slider_cube_revolute_left)
    slider_value_gripper_left = p.readUserDebugParameter(slider_cube_gripper_left)
    
    slider_value_revolute_right = p.readUserDebugParameter(slider_cube_revolute_right)
    slider_value_gripper_right = p.readUserDebugParameter(slider_cube_gripper_right)
    
    
    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_left_Id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_prismatic_left,
                                    physicsClientId=physics_client_id)
                                    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_right_Id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_prismatic_right,
                                    physicsClientId=physics_client_id)
      
    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_left_Id, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_revolute_left,
                                    physicsClientId=physics_client_id)
                                    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_left_Id, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_gripper_left,
                                    physicsClientId=physics_client_id)
                                    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_left_Id, jointIndex=3, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_gripper_left,
                                   physicsClientId=physics_client_id)
                                    
                                    
                                    
                                    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_right_Id, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_revolute_right,
                                    physicsClientId=physics_client_id)
                                    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_right_Id, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_gripper_right,
                                    physicsClientId=physics_client_id)
                                    
    p.setJointMotorControl2(bodyUniqueId=cube_prismatic_right_Id, jointIndex=3, controlMode=p.POSITION_CONTROL, targetPosition=slider_value_gripper_right,
                                    physicsClientId=physics_client_id)
                                 
                                    
        
    p.stepSimulation(physicsClientId=physics_client_id)  

# disconnect from the bullet environment
p.disconnect()
