import rospy
import tf
import numpy as np
import quaternion
from franka_robot import PandaArm
from franka_robot import franka_kinematics
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped


if __name__ == '__main__':
    
    rospy.init_node('test')
    r = PandaArm()
    # print r.has_gripper
    kin = franka_kinematics(r)

    # fk_pos = kin.forward_position_kinematics()

    # print r.joint_ordered_angles()

    # # print kin.inverse_kinematics(fk_pos[:3],fk_pos[3:])

    # print kin._base_link
    # print r.joint_names()
    # print kin.print_robot_description()
    # # print kin.print_kdl_chain()
    # print kin._arm_chain.getNrOfSegments()
    # for idx in xrange(kin._arm_chain.getNrOfSegments()):
    #     print '* ' + kin._arm_chain.getSegment(idx).getName()


    # for jnt_name in range(len(kin._franka.joints)):
    #         jnt = kin._franka.joints[jnt_name]
    #         print "lim",jnt.limit
    # p2 = r._cartesian_pose
    # jpk = kin.inverse_kinematics(position = p2['position'], orientation = np.asarray([p2['orientation'].x,p2['orientation'].y,p2['orientation'].z,p2['orientation'].w]))
    # print "\nkin_j",jpk
    # print "\nmeas_j", r.joint_ordered_angles()
    # # pose1 = PoseStamped()
    # # pose1.header.frame_id = "panda_link0"

    # print r._cartesian_pose

    # joint_dicts = r.joint_angles()
    # i = 0
    # for n in r.joint_names():
    #     joint_dicts[n] = jpk[i]
    #     i += 1
    # print kin.forward_position_kinematics(joint_dicts)

    wrench1 = WrenchStamped()
    wrench1.header.frame_id = "panda_hand"
    pose2 = PoseStamped()
    pose2.header.frame_id = "panda_link0"
    
    # pub1 = rospy.Publisher("pose1", PoseStamped, queue_size = 1)
    pub2 = rospy.Publisher("pose2", PoseStamped, queue_size = 1)
    wrench_pub = rospy.Publisher("ee_wrench", WrenchStamped, queue_size = 1)

    ee_setter = r.get_frames_interface()


    listener = tf.TransformListener()

    # (trans,rot) = listener.lookupTransform('/panda_K', '/panda_hand', rospy.Time(0))
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/panda_link8', '/panda_hand', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # print type(rot)
        if trans:
            break

    print trans, rot

    rot = np.quaternion(rot[3],rot[0],rot[1],rot[2])
    print rot.x, rot.y, rot.z, rot.w

    rot = quaternion.as_rotation_matrix(rot)

    trans_mat = np.eye(4)

    trans_mat[:3,:3] = rot
    trans_mat[:3,3] = np.array(trans)

    print trans_mat

    r.set_EE_frame(trans_mat)    

    # vel = np.asarray([r.joint_velocity(name) for name in r.joint_names()])
    # print "\npykdl ee_vel", kin.jacobian().dot(vel)

    # print "\njac ee_vel", r._jacobian.dot(vel)

    # print "\nmeas ee_vel", r._cartesian_velocity

    # print "\nkdl jac\n", kin.jacobian()

    # print "\nmeas jac\n", r._jacobian

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # break
        # print kin.forward_position_kinematics()
        # argument = r.joint_angles()
        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        # jacobian = np.array(kin.jacobian(argument))
        if not r.in_safe_state():
            print r.get_robot_status()
            if r.error_in_current_state():
                print r.what_errors()
            break
        print "-----------------"
        print ""
        # print jacobian
        # p1 = kin.forward_position_kinematics()

        # pose1.header.stamp = rospy.Time.now()
        # pose1.pose.position.x = p1[0]
        # pose1.pose.position.y = p1[1]
        # pose1.pose.position.z = p1[2]
        # # quaternion = tf.transformations.quaternion_from_euler(p1[3], p1[0], p1[0])
        # pose1.pose.orientation.x = p1[3]
        # pose1.pose.orientation.y = p1[4]
        # pose1.pose.orientation.z = p1[5]
        # pose1.pose.orientation.w = p1[6]
        # print kin.forward_velocity_kinematics()#_cartesian_effort
        print r.joint_ordered_angles()

        # pub1.publish(pose1)
        # print kin.jacobian()
        # print r._jacobian.dot(np.asarray([r.joint_velocity(name) for name in r.joint_velocities()]))
        print ""

        # print r._cartesian_velocity
        # print r._jacobian
        p2 = r._cartesian_pose
        # p2 = np.hstack([p2['position'],p2['orientation']])
        print "ik", kin.inverse_kinematics(position = p2['position'], orientation = np.asarray([p2['orientation'].x,p2['orientation'].y,p2['orientation'].z,p2['orientation'].w]))

        # print p2

        pose2.header.stamp = rospy.Time.now()
        pose2.pose.position.x = p2['position'][0]
        pose2.pose.position.y = p2['position'][1]
        pose2.pose.position.z = p2['position'][2]

        # quaternion = tf.transformations.quaternion_from_euler(p2[3], p2[0], p2[0])
        pose2.pose.orientation.x = p2['orientation'].x
        pose2.pose.orientation.y = p2['orientation'].y
        pose2.pose.orientation.z = p2['orientation'].z
        pose2.pose.orientation.w = p2['orientation'].w

        pub2.publish(pose2)


        wrench = r._cartesian_effort
        wrench = r._stiffness_frame_effort

        # print wrench
        wrench1.header.stamp = rospy.Time.now()
        wrench1.wrench.force.x = wrench['force'][0]
        wrench1.wrench.force.y = wrench['force'][1]
        wrench1.wrench.force.z = wrench['force'][2]

        wrench1.wrench.torque.x = wrench['torque'][0]
        wrench1.wrench.torque.y = wrench['torque'][1]
        wrench1.wrench.torque.z = wrench['torque'][2]

        wrench_pub.publish(wrench1)

        print "-----------------"
        
        # diff = kin.jacobian() - r._jacobian  

        # print diff > 0.005

        print "-----------------"
        print "-----------------"

        # break
        rate.sleep()

    