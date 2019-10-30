import rospy
from franka_robot import FrankaArm
from franka_robot import franka_kinematics

if __name__ == '__main__':
    
    rospy.init_node('test')
    r = FrankaArm()
    # print r.has_gripper
    kin = franka_kinematics(r)
    # print kin._base_link
    # print r.joint_names()
    # print kin.print_robot_description()
    # # print kin.print_kdl_chain()
    # print kin._arm_chain.getNrOfSegments()
    # for idx in xrange(kin._arm_chain.getNrOfSegments()):
    #     print '* ' + kin._arm_chain.getSegment(idx).getName()



    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print kin.forward_position_kinematics()
        # argument = r.joint_angles()
        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        # jacobian = np.array(kin.jacobian(argument))

        print "-----------------"
        print ""
        # print jacobian
        print kin.forward_position_kinematics()
        print ""

        print r._cartesian_pose
        print "-----------------"
        print "-----------------"
        print "-----------------"

        # break
        rate.sleep()

    