from panda_robot import PandaArm
import rospy
# from franka_tools import FrankaFramesInterface
# import tf
import numpy as np
# import quaternion

if __name__ == '__main__':

    rospy.init_node('test')
    r = PandaArm(reset_frames = False)

    angles = r.joint_angles()

    names = r.joint_names()

    vals = [0.000,
            -0.785,
            0.0,
            -1.9,
            0.0,
            1.57,
            0.785]

    def convert_to_dict(val):
        retval = {}
        for n, name in enumerate(names):
            retval[name] = val[n]
        return retval

    def sendto(values):
        r.move_to_joint_positions(convert_to_dict(values))

        print("err" , np.asarray(values) - np.asarray(r.joint_ordered_angles()))

    cm = r.get_controller_manager()

    cms = cm.stop_controller
    cmstart = cm.start_controller