import rospy
# import tf
# import numpy as np
# import quaternion
from panda_robot import PandaArm
# from franka_robot import franka_kinematics
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped


if __name__ == '__main__':
    
    rospy.init_node('test')
    r = PandaArm()

    wrench1 = WrenchStamped()
    wrench1.header.frame_id = "panda_link0"

    wrench_pub = rospy.Publisher("ee_wrench", WrenchStamped, queue_size = 1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not r.in_safe_state():
            print r.get_robot_status()
            if r.error_in_current_state():
                print r.what_errors()
            break
        print "-----------------"
        print ""
        
        print ""
        wrench = r.tip_state()

        wrench1.header.stamp = rospy.Time.now()
        wrench1.wrench.force.x = wrench['force'][0]
        wrench1.wrench.force.y = wrench['force'][1]
        wrench1.wrench.force.z = wrench['force'][2]

        wrench1.wrench.torque.x = wrench['torque'][0]
        wrench1.wrench.torque.y = wrench['torque'][1]
        wrench1.wrench.torque.z = wrench['torque'][2]

        wrench_pub.publish(wrench1)

        print "-----------------"

        print "-----------------"
        print "-----------------"

        rate.sleep()
