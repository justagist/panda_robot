#! /usr/bin/env python

# /***************************************************************************

# 
# @package: panda_robot
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2020, Saif Sidhik
# Copyright (c) 2013-2014, Rethink Robotics
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
    @info
        Interface class for the Franka Robot integrated with gripper handling, 
        low-level controller commanding, motion controlling, trajectory controlling,
        controller manager, frames interface, kinematics, dynamics, etc.

"""

import argparse
import copy
import rospy
import logging
import numpy as np
import quaternion
import franka_interface
from panda_kinematics import PandaKinematics


def compute_omg(q1, q2):
    return 2.*compute_log( quat_mult( quat_convert(q1), quat_conj( quat_convert(q2) ) ) )
    

# ----- TO CHANGE WHEN MOVING TO AML:
# logger
# add ArmInterface as parent class
# from aml_math.quaternion_utils import compute_omg 
# ft smoother subscriber and callback, also tip_state_callback
# ft reading for external ft sensor (in state callback, _configure)
# camera sensor (in state callback, _configure)

class PandaArm(franka_interface.ArmInterface):

    def __init__(self, limb = None, on_state_callback = None, reset_frames = True):

        self._logger = logging.getLogger(__name__)

        self._arm_configured = False # ----- don't update robot state value in this class until robot is fully configured

        # Parent constructor
        franka_interface.ArmInterface.__init__(self)

        self._jnt_limits = [ {'lower': self.get_joint_limits().position_lower[i], 
                             'upper': self.get_joint_limits().position_upper[i]} 
                             for i in range(len(self.joint_names())) ]

        # number of joints
        self._nq = len(self._jnt_limits)
        # number of control commands
        self._nu = len(self._jnt_limits)

        self._configure(on_state_callback)

        self._tuck = [self._neutral_pose_joints[j] for j in self._joint_names]

        self._untuck = self._tuck

        self._q_mean = np.array([0.5 * (limit['lower'] + limit['upper']) for limit in self._jnt_limits])

        self._franka_robot_enable_interface = franka_interface.RobotEnable(self._params)

        if not self._franka_robot_enable_interface.is_enabled():
            self._franka_robot_enable_interface.enable()

        # this will be useful to compute ee_velocity using finite differences
        self._ee_pos_old, self._ee_ori_old = self.ee_pose()
        self._time_now_old = self.time_in_seconds()

        if reset_frames:
            self.set_EE_frame_to_link('panda_hand' if self.has_gripper else 'panda_link8')

    def enable_robot(self):
        # if not self.get_robot_params()._in_sim:
        self._franka_robot_enable_interface.enable()


    def _configure(self, on_state_callback):

        if on_state_callback:
            self._on_state_callback = on_state_callback
        else:
            self._on_state_callback = lambda m: None

        self._configure_gripper(self.get_robot_params().get_gripper_joint_names())

        if self.get_robot_params()._in_sim:
            self._frames_interface = None # Frames interface is not implemented for simulation controller

        self._kinematics = PandaKinematics(self)

        # gravity + feed forward torques
        self._h = [0. for _ in range(self._nq)]

        # self.set_command_timeout(0.2)

        self._transform_ft_vals = False

        self._tip_state = {}

        self._arm_configured = True

        
    def _configure_gripper(self, gripper_joint_names):
        self._gripper = franka_interface.GripperInterface(ns = self._ns, gripper_joint_names = gripper_joint_names)
        if not self._gripper.exists:
            self._gripper = None
            return


    def get_gripper(self):
        """
        :return: gripper instance
        :rtype: GripperInterface

        """
        return self._gripper

    @property
    def has_gripper(self):
        """
        :return: True if gripper is initialised, else False
        :rtype: bool    

        """
        return self._gripper is not None

    def tuck(self):
        self._logger.warning("PandaArm: tuck function NOT IMPLEMENTED")
        self.untuck()

    def untuck(self):
        """
            Move to neutral pose (using trajectory controller)  
        """
        self.move_to_neutral()

    def gripper_state(self):
        gripper_state = {}

        if self._gripper:
            gripper_state['position'] = self._gripper.joint_ordered_positions()
            gripper_state['force'] = self._gripper.joint_ordered_efforts()

        return gripper_state

    def set_gripper_speed(self, speed):

        if self._gripper:
            self._gripper.set_velocity(speed)

    def _update_tip_state(self, tipstate_msg):
        tip_state = {}
        
        time = tipstate_msg.timestamp
        tip_state['position'] = tipstate_msg.pose['position']
        ori = tipstate_msg.pose['orientation']
        force = tipstate_msg.effort['force'] 
        torque = tipstate_msg.effort['torque']

        tip_state['orientation'] = np.asarray([ori.w, ori.x, ori.y, ori.z])
        tip_state['linear_vel'] = tipstate_msg.velocity['linear']
        tip_state['angular_vel'] = tipstate_msg.velocity['angular']
        tip_state['time'] = {'secs':time.secs, 'nsecs':time.nsecs}

        # ----- transform ft to right hand frame ('/right_hand')

        if self._transform_ft_vals:
            rotation_mat = quaternion.as_rotation_matrix(np.quaternion(ori.w, ori.x, ori.y, ori.z))
            tip_state['force'] = np.dot(rotation_mat,np.asarray([-force[0], -force[1], -force[2]]))
            tip_state['torque'] = np.dot(rotation_mat,np.asarray([-torque[0], -torque[1], -torque[2]]))            
        else:
            tip_state['force'] = np.asarray([-force[0], -force[1], -force[2]])
            tip_state['torque'] = np.asarray([-torque[0], -torque[1], -torque[2]])
        tip_state['valid'] = True

        self._tip_state = copy.deepcopy(tip_state)


    def enable_force_torque_transform_to_base_frame(self, boolval=True):
        """
        Enable transformation of force vector to base frame

        :param boolval: set True to transform forces to base frame
        :type boolval: bool
        """
        self._transform_ft_vals = boolval

    def _update_state(self):

        now = rospy.Time.now()

        joint_angles = self.angles()
        joint_velocities = self.velocities()
        joint_efforts = self.joint_efforts()
        tip_state = self.tip_state()

        joint_names = self.joint_names()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        state = {}
        state['position'] = joint_angles
        state['velocity'] = joint_velocities
        state['effort'] = np.array(to_list(joint_efforts))
        state['jacobian'] = self.jacobian(None)
        state['inertia'] = self.inertia(None)
        state['tip_state'] = tip_state

        self._h = self.gravity_comp() # update record of gravity compensation torques

        state['timestamp'] = {'secs': now.secs, 'nsecs': now.nsecs}
        try:
            state['ee_point'], state['ee_ori'] = self.ee_pose()
        except:
            pass

        try:
            tmp = state['jacobian'].dot(state['velocity'])
            state['ee_vel'], state['ee_omg'] = tmp[:3], tmp[3:]#self.ee_velocity()
        except:
            pass

        state['ft_reading'] = None

        state['gripper_state'] = self.gripper_state()

        return state

    def angles(self, include_gripper=False):
        """
        :return: current joint positions
        :rtype: [float]

        :param include_gripper: if True, append gripper joint positions to list
        :type include_gripper: bool
        """
        joint_angles = self.joint_angles()

        joint_names = self.joint_names()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        all_angles = to_list(joint_angles)

        if include_gripper and self._gripper:
            all_angles += self._gripper.joint_ordered_positions()

        return np.array(all_angles)

    def joint_limits(self):
        """
        :return: joint limits
        :rtype: [{'lower': float, 'upper': float}]
        """
        return self._jnt_limits

    def velocities(self, include_gripper=False):
        """
        :return: current joint velocities
        :rtype: [float]

        :param include_gripper: if True, append gripper joint velocities to list
        :type include_gripper: bool
        """
        joint_velocities = self.joint_velocities()

        joint_names = self.joint_names()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        all_velocities = to_list(joint_velocities)

        if include_gripper and self._gripper:
            all_velocities += self._gripper.joint_ordered_velocities()

        return np.array(all_velocities)

    def q_mean(self):
        """
        :return: mean of joint limits
        :rtype: [float]
        """
        return self._q_mean

    def n_cmd(self):
        """
        :return: number of control commands (normally same as number of joints)
        :rtype: int
        """
        return self._nu

    def n_joints(self):
        """
        :return: number of joints
        :rtype: int
        """
        return self._nq

    def state(self):
        """
        :return: robot state as a dictionary
        :rtype: dict {str: obj}
        """
        return self._state

    def tip_state(self):
        """
        :return: tip_state dictionary
        :rtype: dict {str: obj}
        """
        return self._tip_state

    def set_arm_speed(self, speed):
        """
            Set joint position speed (for joint trajectory controller [move_to_joint_positions] only)
        """
        self.set_joint_position_speed(speed)

    def _on_joint_states(self, msg):
        """
            Parent callback function is overriden to update robot state of this class
        """
        franka_interface.ArmInterface._on_joint_states(self, msg)

        if self._arm_configured:
            self._state = self._update_state()
            self._on_state_callback(self._state)
            self._update_tip_state(self.tip_states())

    def end_effector_link_name(self):
        """
        :return: name of end-effector frame
        :rtype: str    

        """
        return self._kinematics._tip_link

    def base_link_name(self):
        """
        :return: name of base link frame
        :rtype: str    

        """
        return self._kinematics._base_link

    def exec_gripper_cmd(self, pos, force=None):
        """
        Move gripper joints to the desired width (space between finger joints), while applying
        the specified force (optional)

        :param pos  : desired width [m]
        :param force: desired force to be applied on object [N]
        :type pos   : float
        :type force : float

        :return: True if command was successful, False otherwise.
        @rtype bool
        """
        if self._gripper is None:
            return

        width = min(self._gripper.MAX_WIDTH, max(self._gripper.MIN_WIDTH, pos))

        if force:
            holding_force = min(max(self._gripper.MIN_FORCE, force), self._gripper.MAX_FORCE)

            return self._gripper.grasp(width = width, force = holding_force)

        else:
            return self._gripper.move_joints(width)


    def exec_gripper_cmd_delta(self, pos_delta, force_delta=None):
        raise NotImplementedError("PandaArm: 'exec_gripper_cmd_delta' not implemented")

    def exec_position_cmd(self, cmd):
        """
        Execute position control (raw positions). Be careful while using. Send smooth
        commands

        :param cmd: desired joint postions, ordered from joint1 to joint7 
                        (optionally, give desired gripper width as 8th element of list)
        :type cmd: [float]
        """

        curr_q = self._state['position']

        if len(cmd) > 7:
            gripper_cmd = cmd[7:]
            self.exec_gripper_cmd(*gripper_cmd)

        joint_command = dict(zip(self.joint_names(), cmd[:7]))

        self.set_joint_positions(joint_command)


    def exec_position_cmd_delta(self, cmd):
        """
        Execute position control based on desired change in joint position

        :param cmd: desired joint postion changes, ordered from joint1 to joint7
        :type cmd: [float]
        """
        curr_q = self.joint_angles()
        joint_names = self.joint_names()

        joint_command = dict([(joint, curr_q[joint] + cmd[i]) for i, joint in enumerate(joint_names)])
        self.set_joint_positions(joint_command)


    def move_to_joint_pos_delta(self, cmd):
        """
        Execute motion (trajectory controller) based on desired change in joint position

        :param cmd: desired joint postion changes, ordered from joint1 to joint7
        :type cmd: [float]
        """
        curr_q = self.joint_angles()
        joint_names = self.joint_names()

        joint_command = dict([(joint, curr_q[joint] + cmd[i]) for i, joint in enumerate(joint_names)])

        self.move_to_joint_positions(joint_command)

    def exec_velocity_cmd(self, cmd):
        """
        Execute velocity command at joint level (using internal velocity controller)

        :param cmd: desired joint velocities, ordered from joint1 to joint7
        :type cmd: [float]
        """
        joint_names = self.joint_names()

        velocity_command = dict(zip(joint_names, cmd))

        self.set_joint_velocities(velocity_command)

    def exec_torque_cmd(self, cmd):
        """
        Execute torque command at joint level directly

        :param cmd: desired joint torques, ordered from joint1 to joint7
        :type cmd: [float]
        """
        joint_names = self.joint_names()

        torque_command = dict(zip(joint_names, cmd))

        self.set_joint_torques(torque_command)

    def move_to_joint_position(self, joint_angles):
        """
        Move to joint position specified (using low-level position control)

        :param joint_angles: desired joint positions, ordered from joint1 to joint7
        :type joint_angles: [float]
        """
        self.move_to_joint_positions(dict(zip(self.joint_names(), joint_angles)))

    def ee_pose(self):
        """
        :return: end-effector pose as position and quaternion in global frame 
        :rtype: np.ndarray (pose), np.quaternion (orientation)
        """
        ee_point = np.asarray(self.endpoint_pose()['position'])

        ee_ori = self.endpoint_pose()['orientation']
        ee_ori = np.quaternion(ee_ori.w, ee_ori.x, ee_ori.y, ee_ori.z)

        return ee_point, ee_ori

    def time_in_seconds(self):
        time_now = rospy.Time.now()
        return time_now.secs + time_now.nsecs * 1e-9

    def ee_velocity(self, real_robot=True):
        """
        :return: end effector velocity (linear and angular) computed using finite difference
        :rtype: np.ndarray, np.ndarray

        :param real_robot: if False, computes ee velocity using finite difference 
        :type real_robot : bool

        this is a simple finite difference based velocity computation
        please note that this might produce a bug since self._goal_ori_old gets
        updated only if get_ee_vel is called.
        """

        if real_robot:

            ee_vel = self.endpoint_velocity()['linear']
            ee_omg = self.endpoint_velocity()['angular']

        else:

            time_now_new = self.time_in_seconds()

            ee_pos_new, ee_ori_new = self.ee_pose()

            dt = time_now_new - self._time_now_old

            ee_vel = (ee_pos_new - self._ee_pos_old) / dt

            ee_omg = compute_omg(ee_ori_new, self._ee_ori_old) / dt

            self._goal_ori_old = ee_ori_new
            self._goal_pos_old = ee_pos_new
            self._time_now_old = time_now_new

        return ee_vel, ee_omg

    def forward_kinematics(self, joint_angles=None, ori_type='quat'):
        """
        :return: position and orientaion of end-effector for the current/provided joint angles
        :rtype: np.ndarray, np.ndarray/np.quaternion

        :param joint_angles: joint angles (optional) for which the ee pose is to be computed 
        :type joint_angles : [float]
        :param ori_type: to specify the orientation representation to return
        """
        if joint_angles is None:

            argument = None

        else:

            argument = dict(zip(self.joint_names(), joint_angles))

        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        pose = np.array(self._kinematics.forward_position_kinematics(argument))
        position = pose[0:3][:, None]  # senting as  column vector

        w = pose[6]
        x = pose[3]
        y = pose[4]
        z = pose[5]  # quarternions

        rotation = quaternion.quaternion(w, x, y, z)

        if ori_type == 'mat':

            rotation = quaternion.as_rotation_matrix(rotation)

        elif ori_type == 'eul':

            rotation = quaternion.as_euler_angles(rotation)
        elif ori_type == 'quat':
            pass

        return position, rotation

    def cartesian_velocity(self, joint_angles=None):
        """
        :return: end-effector velocity computed using kdl
        :rtype: np.ndarray

        :param joint_angles: joint angles (optional) 
        :type joint_angles : [float]
        """
        if joint_angles is None:
            argument = None
        else:
            argument = dict(zip(self.joint_names(), joint_angles))

        return np.array(self._kinematics.forward_velocity_kinematics(argument))[0:3]  # only position

    def jacobian(self, joint_angles=None):
        """
        :return: jacobian matrix of robot at current state
        :rtype: np.ndarray

        :param joint_angles: joint angles (optional) for which the jacobian is to be computed 
        :type joint_angles : [float]
        """
        if joint_angles is None:
            argument = None
        else:
            argument = dict(zip(self.joint_names(), joint_angles))

        jacobian = np.array(self._kinematics.jacobian(argument))

        return jacobian

    def inertia(self, joint_angles=None):
        """
        :return: inertia matrix of robot at current state
        :rtype: np.ndarray

        :param joint_angles: joint angles (optional) 
        :type joint_angles : [float]
        """
        if joint_angles is None:
            argument = None
        else:
            argument = dict(zip(self.joint_names(), joint_angles))

        return np.array(self._kinematics.inertia(argument))

    def inverse_kinematics(self, pos, ori=None, seed=None, null_space_goal = None, **kwargs):
        """
        :return: get the joint positions using inverse kinematics from the provided end-effector pose
        :rtype: bool (success), [float]

        :param pos: end-effector position (x,y,z)
        :type pos : [float]
        :param ori: end-effector orientation (quaternion)
        :type ori : [float] or np.quaternion
        :param seed: seed joints to start ik computation
        :type seed: [float]
        :param null_space_goal: null-space joint position if required
        :type null_space_goal: [float]

        kwargs are to avoid breaking of sister classes for arguments that are not used in this class.
        """
        success = False
        soln = None

        if ori:
            # expects a pykdl quaternion which is of type x,y,z,w 
            if isinstance(ori, np.quaternion):
                ori = np.array([ori.x, ori.y, ori.z, ori.w])

        soln = self._kinematics.inverse_kinematics(position=pos, orientation=ori, seed=seed)

        if soln:
            success = True

        return success, soln


def main():
    rospy.init_node("panda_arm_untuck")

    arm = PandaArm(reset_frames = False)

    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument("-u", "--untuck",
        action='store_true', default=False, help="untuck arms")
    
    args = parser.parse_args(rospy.myargv()[1:])
    untuck = args.untuck

    if untuck:
        rospy.loginfo("Untucking arms")
        arm.untuck()
        rospy.loginfo("Finished Untuck")

    #     print "this",arm.state()['ft_reading'][:3]
    #     # print arm.state()['ee_ori']
    #     # arm.state()
    #     # pass
    #     rospy.sleep(0.1)

    # rospy.spin()


if __name__ == '__main__':
    main()