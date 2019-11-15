#! /usr/bin/env python

"""
    Script to test PandaArm and panda_robot package.

    Run script interactively to test different functions.
        python -i env.py

"""

import rospy
import numpy as np
from panda_robot import PandaArm



poses = [[-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01],
         [ 1.34695728e-01, -2.74474940e-01, -2.46027836e-01, -1.19805447e+00, -5.27289847e-05,  2.17926193e+00,  9.10497957e-01],
         [ 1.81297444e-01,  3.94348774e-01, -2.25835923e-01, -1.19416311e+00, -7.51349249e-04,  2.79453565e+00,  8.36526167e-01],
         [ 0.63068724,      0.86207321,     -0.52113169,     -0.95186331,     0.02450696,       2.64150352,      0.5074312 ]]


if __name__ == '__main__':

    rospy.init_node("panda_env")
    
    r = PandaArm(reset_frames = False)
    fi = r.get_frames_interface()
    cm = r.get_controller_manager()

    neutral = r.move_to_neutral
    move_to = r.move_to_joint_position

    g = r.get_gripper()

    # poses = 


