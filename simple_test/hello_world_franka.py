#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

"""

import pybullet as p
from franka_panda_env import PandaEnv

physics_client_id = p.connect(p.GUI)

p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=physics_client_id)

franka_left = PandaEnv(physics_client_id, base_position=(0.0, 0.0, 0.0))
franka_left.show_sliders()

# Rotation on Z with 3.14159 radians, displacement on X with 1.0 meter
franka_right = PandaEnv(physics_client_id, base_position=(1.0, 0.0, 0.0), base_orientation = (0.0, 0.0, 3.14159))

# infinite loop to move the panda in function of some keyboard events
while True:

    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, physicsClientId=physics_client_id)
    
    # look if there is keyboard event and put it in a dictionary named keys
    keys = p.getKeyboardEvents()
    
    # 'Enter' event = 65309, if so .. break the loop
    if 65309 in keys:
      break

# disconnect from the bullet environment
p.disconnect()
