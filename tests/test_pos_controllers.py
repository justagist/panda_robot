import rospy
import numpy as np
from copy import deepcopy
from panda_robot import PandaArm

"""
    Script to test PandaArm controller command interface.

    NOTE: this code will make the robot go to neutral (with MoveIt or JT action client).
    After hitting Enter, the robot will slowly move (small, slow wavy motion, similar 
    to the franka_ros position controller demo) using position control or impedance 
    control.

    NOTE: this is a very simplified code. In ideal case, the control commands should be
    sent from a controller loop running independently for smooth motion.

    Run script. Then Press Enter when prompted.

"""

if __name__ == '__main__':
    

    rospy.init_node("ctrl_test")
    r = PandaArm()

    rate = rospy.Rate(100)

    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    r.move_to_neutral() # move to neutral pose before beginning

    initial_pose = deepcopy(r.angles())

    input("Hit Enter to Start")
    print("commanding")
    vals = deepcopy(initial_pose)
    count = 0

    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for j, _ in enumerate(vals):
            if j == 4:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        if count%500 == 0:
            print(vals, delta)
            print("\n ----  \n")
            print(" ")


        ## ===== IMPORTANT: ONLY USE ONE CONTROL METHOD AT A TIME IN A LOOP! COMMENT OUT ONE OF THE TWO LINES BELOW ALWAYS!

        r.set_joint_positions_velocities(vals, [0.0 for _ in range(7)]) # uncomment this line for impedance control
        # r.exec_position_cmd(vals) # comment out the above line and uncomment this for position control. NOTE: the provided trajectory is still not fully smooth, so the motion might be slightly jerky 

        ## ==================================================================================================================

        count += 1
        rate.sleep()
