import rospy
from panda_robot import PandaArm
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from copy import deepcopy

vals = []
vels = []
names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

if __name__ == '__main__':
    

    rospy.init_node("test_node")
    r = PandaArm()

    rate = rospy.Rate(100)


    # while len(vals) != 7 and not rospy.is_shutdown():
    #     continue

    initial_pose = deepcopy(r.joint_ordered_angles())

    vals = deepcopy(initial_pose)

    i = 0.1

    r.move_to_neutral()



    # elapsed_time_ = rospy.Duration(0.0)
    # period = rospy.Duration(0.005)

    # count = 0

    raw_input("Hit Enter to Start")
    joints = r.angles()

    limits = r.joint_limits()

    lower_lim = limits[0]['lower']/4
    upper_lim = limits[0]['upper']/4
    print "commanding"

    speed = 20

    start_time = rospy.Time().now().to_sec()

    diff = (upper_lim - lower_lim)/2.0

    offset = joints[0]

    while not rospy.is_shutdown():

        joints[0] = np.sin((rospy.Time().now().to_sec() - start_time)/100.0 * speed)*diff + offset

        r.exec_position_cmd(joints)

        rate.sleep()

        # elapsed_time_ += period

        # delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2
        # # delta = 0.001
        # delta = 0.3*delta
        # # if count%100 == 0:
        # for j in range(len(vals)):
        #     if j == 4:
        #         vals[j] = initial_pose[j] - delta
        #     else:
        #         vals[j] = initial_pose[j] + delta

        # # plt.plot(a)
        # # plt.pause(0.00001)
        # if count%500 == 0:
        #     print vals, delta
        #     print "\n ----  \n"
        #     print " "
        #     print initial_pose

        # vel_dict = r.joint_velocities()
        # vels = [vel_dict[j] for j in names]

        # pos = {}
        # for j in range(len(names)):
        #     pos[names[j]] = vals[j]
        # r.set_joint_positions(pos)

        # count += 1
        # rate.sleep()
        # r.set_joint_positions_velocities(vals, vels)
        
        # if vals[6] >= max_val:
        #     delta = -upd
        # if vals[6] <= min_val:
        #     delta = upd

        # pubmsg = JointCommand()
        # # pubmsg.mode = pubmsg.TORQUE_MODE
        # pubmsg.names = names
        # pubmsg.mode = pubmsg.IMPEDANCE_MODE
        # # pubmsg.position = [0.00020082598954863977, -0.7850038009359614, 0.00015446583697012738, -2.3556103476139536, -0.00048749371177230215, 1.571537698017226, vals[-1]]
        # pubmsg.position = vals
        # # pubmsg.position = [0.00020082598954863977, -0.7850038009359614, 0.00015446583697012738, -2.3556103476139536, -0.00048749371177230215, 1.571537698017226, 0.7845346834179426]
        # # [0.00020082598954863977, -0.7850038009359614, 0.00015446583697012738, -2.3556103476139536, -0.00048749371177230215, 1.571537698017226, 0.7845346834179426]
        # pubmsg.velocity = vels
        # pubmsg.position[6] = pubmsg.position[6] + delta
        # print pubmsg.position[6]
        # flt_msg = Float64(vals[-2])

        # pub2.publish(flt_msg)

        # # # print pubmsg.position
        # # # print vals
        # pub.publish(pubmsg)