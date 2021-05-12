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

base_position_cylinder = [-0.75,0.0,0.52]

#cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 100, collisionMargin = 0.005, useMassSpring=0, useBendingSprings=0, useNeoHookean = 1, NeoHookeanMu = 83200, NeoHookeanLambda = 83200, NeoHookeanDamping = 1000, springElasticStiffness=0.0, springDampingStiffness=0.0, springBendingStiffness=0.0, springDampingAllDirections=0, frictionCoeff=.5, useFaceContact=0, useSelfCollision=0, repulsionStiffness=0.0)

cylinderId = p.loadSoftBody("deformable_object/tetra_cylinder_50_cm.vtk", basePosition = base_position_cylinder, baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 10.0, collisionMargin = 0.01, useNeoHookean = 1, NeoHookeanMu = 83200, NeoHookeanLambda = 83200, NeoHookeanDamping = 1000, frictionCoeff=.5)


while True:

    # look if there is keyboard event and put it in a dictionary named keys
    keys = p.getKeyboardEvents()
    
    # 'Enter' event = 65309, if so .. break the loop
    if 65309 in keys:
      break
      
    p.stepSimulation(physicsClientId=physics_client_id)  

# disconnect from the bullet environment
p.disconnect()
