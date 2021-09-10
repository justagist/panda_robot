"""
    Script to test PandaArm and panda_robot package.

    Run script interactively to test different functions.
        python -i env.py

"""

import rospy
from panda_robot import PandaArm


poses = [[-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01],
         [ 1.34695728e-01, -2.74474940e-01, -2.46027836e-01, -1.19805447e+00, -5.27289847e-05,  2.17926193e+00,  9.10497957e-01],
         [ 1.81297444e-01,  3.94348774e-01, -2.25835923e-01, -1.19416311e+00, -7.51349249e-04,  2.79453565e+00,  8.36526167e-01],
         [ 0.63068724,      0.86207321,     -0.52113169,     -0.95186331,     0.02450696,       2.64150352,      0.5074312 ]]

if __name__ == '__main__':

    rospy.init_node("panda_env")

    r = PandaArm(reset_frames = False) # handle to use methods from PandaArm class
    fi = r.get_frames_interface() # frames interface object for the robot. Test switching EE frames
                                    # How to test:
                                    # 1) open rviz -> add RobotModel (topic 'robot_description')
                                    # 2) set panda_link0 as global fixed frame
                                    # 3) add tf -> disable visualisation of all links except panda_EE
                                    # 4) run this script in terminal in interactive mode
                                    # 5) type $ fi.set_EE_frame_to_link('panda_hand')
                                    #       to move the EE frame to the link. Try different link names.
                                    #       Test the same for the stiffness frame (set_K_frame_to_link)

    cm = r.get_controller_manager() # controller manager object to get controller states and switch controllers, controllers don't have to be switched manually in most cases! The interface automatically chooses the right command depending on the control command sent (r.exec_position_cmd, r.exec_velocity_cmd, r.exec_torque_cmd, r.set_joint_position_velocity)

    kin = r._kinematics # to test the kinematics (not required, can directly query kinematics using  methods in PandaArm)

    g = r.get_gripper() # gripper object. Test using $ g.close(), $ g.open(), $ g.home_joints(), $g.move_joints(0.01), etc.

    neutral = r.move_to_neutral
    move_to = r.move_to_joint_position
    mtc = r.move_to_cartesian_pose

    # In interactive mode, for instance enter
    #               $ neutral()
    #   to make the robot move to neutral pose
    #   or type $ move_to(poses[0])
    #      to move to the first joint pose from the list defined above (make sure robot workspace is free, visualise in panda_simulator or moveit planning before executing to be sure.)

    # movegroup interface instance of the robot. See https://justagist.github.io/franka_ros_interface/DOC.html#pandamovegroupinterface for available methods.
    mvt = r.get_movegroup_interface()
