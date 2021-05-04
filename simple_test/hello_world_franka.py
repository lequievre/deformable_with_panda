#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr


https://github.com/DanielTakeshi/deformable-ravens

"""

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

p.setGravity(0, 0, -10)

p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=physics_client_id)

franka_left = PandaEnv(physics_client_id, base_position=(-1.2, 0.0, 0.0))
franka_left.show_sliders(prefix_name='left_')

# Rotation on Z with 3.14159 radians, displacement on X with 1.0 meter
franka_right = PandaEnv(physics_client_id, base_position=(1.2, 0.0, 0.0), base_orientation = (0.0, 0.0, 3.14159))
franka_right.show_sliders(prefix_name='right_')


planId = p.loadURDF("plane.urdf", [0,0,0])

base_orientation_cylinder = p.getQuaternionFromEuler((0.0, 0.0, 1.5707))


#tableId = p.loadURDF("table/table.urdf")


# Load deformable object
#cylinderId = p.loadSoftBody("deformable_object/cylinder_blender.obj",basePosition=(0.5, 0.0, -2.0), baseOrientation = base_orientation_cylinder)


#cylinderId = p.loadSoftBody("deformable_object/cylinder_centered.obj", basePosition = [0,0,2], baseOrientation=base_orientation_cylinder, scale = 1.0, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=100, springDampingStiffness=.001, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)

clothId = p.loadSoftBody("bunny.obj", basePosition = [0.5,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=100, springDampingStiffness=.001, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1, repulsionStiffness=100.0)


#cylinderId = p.loadURDF("deformable_object/melodie_cylinder.urdf",
#                                   basePosition=(0.0, 0.0, 2.0), #baseOrientation=base_orientation_cylinder, useFixedBase=True, #physicsClientId=physics_client_id)

#cylinderId = p.loadURDF("deformable_object/melodie_cylinder.urdf", useFixedBase=True)

# infinite loop to move the panda in function of some keyboard events
while True:

    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, physicsClientId=physics_client_id)

    franka_left.apply_sliders()
    franka_right.apply_sliders()
    
    # look if there is keyboard event and put it in a dictionary named keys
    keys = p.getKeyboardEvents()
    
    # 'Enter' event = 65309, if so .. break the loop
    if 65309 in keys:
      break

# disconnect from the bullet environment
p.disconnect()
