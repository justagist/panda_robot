#!/usr/bin/env python

# /***************************************************************************

# 
# @package: panda_robot
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2021, Saif Sidhik
# Copyright (c) 2013-2018, Rethink Robotics Inc.
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/


import rospy
import numpy as np
from panda_robot import PandaArm
from franka_dataflow.getch import getch

pos_increment = 0.01
ori_increment = 0.001
def map_keyboard():
    """
        Map keyboard keys to robot joint motion. Keybindings can be 
        found when running the script.
    """

    limb = PandaArm()

    has_gripper = limb.get_gripper() is not None

    # joints = limb.joint_names()

    def set_ee_target(action, value):
    	pos, ori = limb.ee_pose()

    	if action == 'position':
    		pos += value
		status, j_des = limb.inverse_kinematics(pos, ori)
		# print j_des
		if status:
			limb.exec_position_cmd(j_des)

    def set_g(action):
        if has_gripper:
            if action == "close":
                limb.get_gripper().close()
            elif action == "open":
                limb.get_gripper().open()
            elif action == "calibrate":
                limb.get_gripper().calibrate()
    def reset_robot(args):
    	limb.untuck()

    bindings = {
        '5': (set_ee_target, ['position', np.asarray([pos_increment, 0, 0])], "x increase"),
        '2': (set_ee_target, ['position', np.asarray([-pos_increment, 0, 0])], "x decrease"),
        '1': (set_ee_target, ['position', np.asarray([0, pos_increment, 0])], "y increase"),
        '3': (set_ee_target, ['position', np.asarray([0, -pos_increment, 0])], "y decrease"),
        '7': (set_ee_target, ['position', np.asarray([0, 0, pos_increment])], "z increase"),
        '4': (set_ee_target, ['position', np.asarray([0, 0, -pos_increment])], "z decrease"),
        'r': (reset_robot, [None], "reset to neutral pose")
     }
    if has_gripper:
        bindings.update({
        '8': (set_g, "close", "close gripper"),
        '9': (set_g, "open", "open gripper"),
        'i': (set_g, "calibrate", "calibrate gripper")
        })
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                if c == '8' or c == 'i' or c == '9':
                    cmd[0](cmd[1])
                    print("command: %s" % (cmd[2],))
                else:
                    #expand binding to something like "set_j(right, 'j0', 0.1)"
                    cmd[0](*cmd[1])
                    print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def main():
    """Panda Robot IK Example: End-effector Keyboard Control

    Use your dev machine's keyboard to control robot end-effector position.
    """

    print("Initializing node... ")
    rospy.init_node("fri_example_joint_position_keyboard")
    print("Getting robot state... ")

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()



