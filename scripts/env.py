#! /usr/bin/env python

"""
    Script to test PandaArm and panda_robot package.

    Run script interactively to test different functions.
        python -i env.py

"""

import rospy
import numpy as np
from panda_robot import PandaArm


if __name__ == '__main__':

    rospy.init_node("panda_env")
    
    r = PandaArm(reset_frames = False)
    fi = r.get_frames_interface()
    cm = r.get_controller_manager()


