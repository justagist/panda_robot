
import rospy
import numpy as np
from panda_robot import PandaArm


if __name__ == '__main__':

    rospy.init_node("controller_env")
    
    r = PandaArm()
    fi = r.get_frames_interface()
    cm = r.get_controller_manager()

    while True:
        # robot should be holding the same position in joint impedance mode
        vels = r.joint_velocities()
        r.set_joint_positions_velocities(r.joint_ordered_angles(), [vels[j] for j in r.joint_names()])


