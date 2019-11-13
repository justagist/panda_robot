from panda_robot import PandaArm
import rospy 
from franka_tools import FrankaFramesInterface
import tf
import numpy as np
import quaternion

if __name__ == '__main__':
    
    rospy.init_node('test')
    r = PandaArm(reset_frames = False)

    angles = r.joint_angles()

    # print r.joint_ordered_angles()

    names = r.joint_names()

    vals = [0.000,
            -0.785,
            0.0,
            -1.9,
            0.0,
            1.57,
            0.785]

    # vals2 = [2.1938958153278065e-05, -1.3450673457612599, -2.465007235841199e-05, -3.071840196444259, 0.00047313619350308755, 1.7241588331793718, 0.7851552560080668]

    def convert_to_dict(val):
        retval = {}
        for n in range(len(names)):
            retval[names[n]] = val[n]
        return retval

    def sendto(values):
        r.move_to_joint_positions(convert_to_dict(values))

        print "err" , np.asarray(values) - np.asarray(r.joint_ordered_angles())

    # r.get_controller_manager().load_controller('position_joint_trajectory_controller')

    # r.move_to_joint_positions(convert_to_dict(vals))
    cm = r.get_controller_manager()

    cms = cm.stop_controller
    cmstart = cm.start_controller
    # r.move_to_joint_positions(vals)

    # ee_setter = r.get_frames_interface()

    # r.set_EE_frame_to_link("/panda_link2")


    # listener = tf.TransformListener()

    # # (trans,rot) = listener.lookupTransform('/panda_K', '/panda_hand', rospy.Time(0))
    # while not rospy.is_shutdown():
    #     try:
    #         (trans,rot) = listener.lookupTransform('/panda_link8', '/panda_hand', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    #     # print type(rot)
    #     if trans:
    #         break

    # print trans, rot

    # rot = np.quaternion(rot[3],rot[0],rot[1],rot[2])
    # print rot.x, rot.y, rot.z, rot.w

    # rot = quaternion.as_rotation_matrix(rot)

    # trans_mat = np.eye(4)

    # trans_mat[:3,:3] = rot
    # trans_mat[:3,3] = np.array(trans)

    # print trans_mat

    # r.set_EE_frame(trans_mat)    



    # print rot.x

    # print ee_setter.get_EE_frame()