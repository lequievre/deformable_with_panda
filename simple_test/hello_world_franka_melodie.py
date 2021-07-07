#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr


https://github.com/DanielTakeshi/deformable-ravens

"""

import time
from time import sleep
import pybullet as p
from franka_panda_env import PandaEnv

import os, inspect

# Get and print infos about current and parent directory
currentframe = inspect.currentframe()
print("current file name={0}".format(inspect.getfile(inspect.currentframe())))
print("abs path of file={0}".format(os.path.abspath(inspect.getfile(inspect.currentframe()))))
print("current dir of file={0}".format(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))))

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, currentdir)

print("current dir of file={0}".format(currentdir))

physics_client_id = p.connect(p.GUI)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

#p.setPhysicsEngineParameter(fixedTimeStep = 0.02, numSolverIterations = 200, useSplitImpulse = 1, erp = 0.1, solverResidualThreshold = 0.001, sparseSdfVoxelSize = 0.25)
#p.setRealTimeSimulation(0)
#p.setTimeStep(0.001, physics_client_id)
#p.setPhysicsEngineParameter(sparseSdfVoxelSize = 0.25)


p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=physics_client_id)

franka_left = PandaEnv(physics_client_id, base_position=(-1.4, 0.0, 0.0))

#franka_left.debug_gui()

# Rotation on Z with 3.14159 radians, displacement on X with 1.0 meter
franka_right = PandaEnv(physics_client_id, base_position=(1.4, 0.0, 0.0), base_orientation = (0.0, 0.0, 3.14159))


planId = p.loadURDF("plane.urdf", [0,0,0])

base_orientation_cylinder = p.getQuaternionFromEuler((0.0, 1.57, 0.0))

base_position_cylinder = [-0.75,0.0,0.0]

# Load deformable object
#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_2_5_mm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, mass = 0.02, useNeoHookean = 1, NeoHookeanMu = 96.1, NeoHookeanLambda = 144.2, NeoHookeanDamping = 0.01, useSelfCollision = 1, collisionMargin = 0.001, frictionCoeff = 0.5)

cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_1_25_mm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, mass = 0.02, useNeoHookean = 1, NeoHookeanMu = 85.0, NeoHookeanLambda = 130.0, NeoHookeanDamping = 0.01, useSelfCollision = 1, collisionMargin = 0.001, frictionCoeff = 0.8)


#p.setPhysicsEngineParameter(fixedTimeStep = 0.001, physicsClientId = physics_client_id, numSolverIterations = 200, useSplitImpulse = 1, erp = 0.1, solverResidualThreshold = 0.001, sparseSdfVoxelSize = 0.25)
p.setTimeStep(0.001, physicsClientId = physics_client_id)

texUid = p.loadTexture("deformable_object/texture/texture_frite.png")
p.changeVisualShape(cylinderId, -1, textureUniqueId=texUid)


for i in range(10):
	p.stepSimulation(physicsClientId=physics_client_id) 




data = p.getMeshData(cylinderId, -1, flags=p.MESH_DATA_SIMULATION_MESH)


#p.createSoftBodyAnchor(cylinderId ,0,franka_right.robot_id,-1, [0.0,0.0,0])
#p.createSoftBodyAnchor(cylinderId ,1,franka_right.robot_id,-1, [0.0,0.0,0])


# n° 56 right
# n°4 left
pose_cylinder_right = list(data[1][65])  # n° 65 right
pose_cylinder_right[2]+=0.05
#pose_cylinder_right[0]-=0.08
pose_cylinder_left = list(data[1][5]) # n°5 left
pose_cylinder_left[2]+=0.06
pose_cylinder_left[0]+=0.08

"""
pose_cylinder_right = list(data[1][0])  # n° 56 right
pose_cylinder_right[2]+=0.08
pose_cylinder_left = list(data[1][len(range(data[0]))-1]) # n°4 left
pose_cylinder_left[2]+=0.08
"""


uid_right = p.addUserDebugText("*", pose_cylinder_right, textColorRGB=[0,0,0])
uid_left = p.addUserDebugText("*", pose_cylinder_left, textColorRGB=[0,0,0])

jointPoses_right = p.calculateInverseKinematics(franka_right.robot_id, franka_right.end_eff_idx, pose_cylinder_right)


#franka_right.apply_action(jointPoses_right[0:7])
#p.stepSimulation(physicsClientId=physics_client_id)

jointPoses_left = p.calculateInverseKinematics(franka_left.robot_id, franka_left.end_eff_idx, pose_cylinder_left, franka_left.list_lower_limits, franka_left.list_upper_limits, franka_left.list_ranges, franka_left.list_rest_pos)

#franka_left.apply_action(jointPoses_left[0:7])
#p.stepSimulation(physicsClientId=physics_client_id)

franka_left.show_sliders(prefix_name='left_', joint_values=jointPoses_left)
franka_right.show_sliders(prefix_name='right_', joint_values=jointPoses_right)


print("joint poses right = ", jointPoses_right)
print("joint poses left = ", jointPoses_left)

  
print("position right = ",data[1][65])
print("position left = ",data[1][5])
  
  
"""print("--------------")
  print("data=",data)
  print(data[0])
  print(data[1])
  """
"""
text_uid = []
for i in range(data[0]):
    pos = data[1][i]
    uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
    text_uid.append(uid)
"""

# infinite loop to move the panda in function of some keyboard events
while True:

    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, physicsClientId=physics_client_id)

    franka_left.apply_sliders()
    franka_right.apply_sliders()
    
    # look if there is keyboard event and put it in a dictionary named keys
    keys = p.getKeyboardEvents()
    
    #print(keys)
    
    # 'Enter' event = 65309, if so .. break the loop
    if 65309 in keys:
      break
      
    if 111 in keys:  # 'o' for the left gripper
      franka_left.close_gripper()
		
    if 112 in keys: # Grasp the object by using force (with letter 'p' = 112) for right gripper
      franka_right.close_gripper()
      
      
    p.stepSimulation(physicsClientId=physics_client_id)  
    

# disconnect from the bullet environment
p.disconnect()
