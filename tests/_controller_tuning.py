import rospy
from panda_robot import PandaArm
import numpy as np
from copy import deepcopy

# test file; used for tuning impedance controller gains

vals = []
vels = []
names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

if __name__ == '__main__':


    rospy.init_node("test_node", disable_signals = True)
    r = PandaArm()

    rate = rospy.Rate(100)

    initial_pose = deepcopy(r.joint_ordered_angles())

    vals = deepcopy(initial_pose)

    i = 0.1

    r.move_to_neutral()

    input("Hit Enter to Start")
    joints = r.angles()

    limits = r.joint_limits()

    lower_lim = limits[0]['lower']/4
    upper_lim = limits[0]['upper']/4
    print("commanding")

    speed = 10

    start_time = rospy.Time().now().to_sec()

    diff = (upper_lim - lower_lim)/2.0

    offset = joints[0]

    while not rospy.is_shutdown():

        joints[0] = np.sin((rospy.Time().now().to_sec() - start_time)/100.0 * speed)*diff + offset

        r.set_joint_positions_velocities(joints, r.velocities())

        rate.sleep()
