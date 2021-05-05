import rospy
# import tf
# import numpy as np
# import quaternion
from panda_robot import PandaArm
# from franka_robot import franka_kinematics
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

"""
:info:
    Visualise the end-effector wrench in the base frame of the robot. This is
    the wrench applied BY the robot to OPPOSE external force. Theoretically,
    this should be equal and opposite to the external force acting on it.

    Steps:
        - Run this code.
        - Open RViz (set Global frame to "base") and add a Display for RobotModel.
        - Add another Display for WrenchStamped with topic "test_ee_wrench".

    NOTE: You might have to change the scale value of force and torque arrows
    in RViz. 0.1 is good for visualisation.

    NOTE: In this demo, the wrench is shown at the end-effector of the robot,
    but in reality the computation is done with respect to the robot's base
    frame. You can display in this (actual) frame by changing line #32.
"""

if __name__ == '__main__':
    
    rospy.init_node('test')
    r = PandaArm()

    wrench1 = WrenchStamped()
    wrench1.header.frame_id = "panda_EE" # change this to "panda_link0" to visualise wrench in the original frame

    wrench_pub = rospy.Publisher("test_ee_wrench", WrenchStamped, queue_size = 1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not r.in_safe_state():
            print(r.get_robot_status())
            if r.error_in_current_state():
                print(r.what_errors())
            break
        # print("-----------------")
        # print("")
        
        # print("")
        wrench = r.tip_state()

        wrench1.header.stamp = rospy.Time.now()
        wrench1.wrench.force.x = wrench['force'][0]
        wrench1.wrench.force.y = wrench['force'][1]
        wrench1.wrench.force.z = wrench['force'][2]

        wrench1.wrench.torque.x = wrench['torque'][0]
        wrench1.wrench.torque.y = wrench['torque'][1]
        wrench1.wrench.torque.z = wrench['torque'][2]

        wrench_pub.publish(wrench1)

        # print("-----------------")

        # print("-----------------")
        # print("-----------------")

        rate.sleep()
