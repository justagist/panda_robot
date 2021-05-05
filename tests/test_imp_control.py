
import rospy
from panda_robot import PandaArm

"""
:info:
    Robot should be holding the same (current) position in joint impedance mode.
    Test by pushing robot. It should be 'springy'. 
    
    Warning: The default torque and force safety limits of the robot is disabled
        for this test. Otherwise pushing a little hard will trigger safety stop
        from libfranka.
"""

def _clean_shutdown():
    # reset collision thresholds to libfranka defaults when demo ends
    r.set_collision_threshold()

if __name__ == '__main__':

    rospy.init_node("controller_env")

    r = PandaArm()

    force_threshold = [100,100,100,100,100,100] # cartesian force threshold
    torque_threshold = [100,100,100,100,100,100,100] # joint torque threshold
    
    # increase collision detection thresholds for testing
    r.set_collision_threshold(joint_torques = torque_threshold, cartesian_forces = force_threshold)

    rospy.on_shutdown(_clean_shutdown)

    initial_pose = r.joint_ordered_angles()
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # robot should be holding the same position in joint impedance mode
        vels = r.joint_velocities()
        r.set_joint_positions_velocities(initial_pose, [vels[j] for j in r.joint_names()]) # impedance control command (see documentation at )
        rate.sleep()
