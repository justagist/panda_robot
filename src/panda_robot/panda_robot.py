#!/usr/bin/env python

# /***************************************************************************

#
# @package: panda_robot
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2021, Saif Sidhik
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

import copy
import rospy
import logging
import argparse
import quaternion
import numpy as np
import franka_interface
from .utils.math_utils import compute_omg
from .panda_kinematics import PandaKinematics


def time_in_seconds():
    time_now = rospy.Time.now()
    return time_now.secs + time_now.nsecs * 1e-9


class PandaArm(franka_interface.ArmInterface):
    """
        Methods from :py:class:`franka_interface.ArmInterface` are also available to objects of this class.

        :bases: :py:class:`franka_interface.ArmInterface`

        :param on_state_callback: optional callback function to run on each state update
        :param reset_frames: if True, EE frame is reset using :py:class:`franka_interface.ArmInteface`
                (using `franka_interface.ArmInterface <https://justagist.github.io/franka_ros_interface/DOC.html#arminterface>`_ and `franka_tools.FrankaFramesInterface <https://justagist.github.io/franka_ros_interface/DOC.html#frankaframesinterface>`_).

    """

    def __init__(self, on_state_callback=None, reset_frames=True):
        """
            Constructor class.  Functions from `franka_interface.ArmInterface <https://justagist.github.io/franka_ros_interface/DOC.html#arminterface>`_

            :param on_state_callback: optional callback function to run on each state update
            :param reset_frames: if True, EE frame is reset using :py:class:`franka_interface.ArmInteface`
                (using :py:class:`franka_interface.ArmInterface` and :py:class:`franka_tools.FrankaFramesInterface`).
        """

        self._logger = logging.getLogger(__name__)

        # ----- don't update robot state value in this class until robot is fully configured
        self._arm_configured = False

        # Parent constructor
        franka_interface.ArmInterface.__init__(self)

        self._jnt_limits = [{'lower': self.get_joint_limits().position_lower[i],
                             'upper': self.get_joint_limits().position_upper[i]}
                            for i in range(len(self.joint_names()))]

        # number of joints
        self._nq = len(self._jnt_limits)
        # number of control commands
        self._nu = len(self._jnt_limits)

        self._configure(on_state_callback)

        self._tuck = [self._neutral_pose_joints[j] for j in self._joint_names]

        self._untuck = self._tuck

        self._q_mean = np.array(
            [0.5 * (limit['lower'] + limit['upper']) for limit in self._jnt_limits])

        self._franka_robot_enable_interface = franka_interface.RobotEnable(
            self._params)

        if not self._franka_robot_enable_interface.is_enabled():
            self._franka_robot_enable_interface.enable()

        # this will be useful to compute ee_velocity using finite differences
        self._ee_pos_old, self._ee_ori_old = self.ee_pose()
        self._time_now_old = time_in_seconds()

        if reset_frames:
            self.reset_EE_frame()

    def enable_robot(self):
        """
            Re-enable robot if stopped due to collision or safety.
        """
        self._franka_robot_enable_interface.enable()

    def _configure(self, on_state_callback):

        if on_state_callback:
            self._on_state_callback = on_state_callback
        else:
            self._on_state_callback = lambda m: None

        self._configure_gripper(
            self.get_robot_params().get_gripper_joint_names())

        if self.get_robot_params()._in_sim:
            # Frames interface is not implemented for simulation controller
            self._frames_interface = None
        
        self.set_up_kinematics_interface()

        self._tip_state = {}

    def set_up_kinematics_interface(self):
        self._arm_configured = False
        rospy.sleep(0.5) # seem to need some delay, otherwise error: state update happens with non-existent _kinematics object
        self._kinematics = None
        if self.get_robot_params()._in_sim:
            self._kinematics = PandaKinematics(
                self, self.name + ('_hand' if self.has_gripper else '_link8'))
        else:
            # match the kinematics end-effector with the libfranka-defined EE by adding segments to kdl chain
            # if self.has_gripper:
            ee_frame_name = self.name + "_EE"
            F_T_NE = np.asarray(self._F_T_NE).reshape(4, 4, order="F")
            NE_T_EE = np.asarray(self._NE_T_EE).reshape(4, 4, order="F")
            ee_conf = [{
                "child_name": self.name+"_NE",
                "origin_pos": F_T_NE[:3, 3],
                "origin_ori":F_T_NE[:3, :3],
                "joint_name":"fixed_NE_jnt",
                "parent_name":self.name+"_link8"
            },
                {
                "child_name": self.name+"_EE",
                "origin_pos": NE_T_EE[:3, 3],
                "origin_ori":NE_T_EE[:3, :3],
                "joint_name":"fixed_EE_jnt",
                "parent_name":self.name+"_NE"
            }
            ]
            self._kinematics = PandaKinematics(
                self, ee_frame_name, additional_segment_config=ee_conf)
            # else:
            #     self._kinematics = PandaKinematics(
            #         self, self.name + '_link8')
        # rospy.sleep(0.5)
        self._arm_configured = True

    def set_EE_frame(self, frame):
        """
        .. note:: This method is not available in simulated environment (when using PandaSimulator).

        Set new EE frame based on the transformation given by 'frame', which is the 
        transformation matrix defining the new desired EE frame with respect to the 
        nominal end-effector frame (NE_T_EE).
        Motion controllers are stopped and restarted for switching. Also resets the 
        kinematic chain for PyKDL IK/FK computations.

        :type frame: [float (len = 16)] (or) numpy.ndarray (4x4) 
        :param frame: transformation matrix of new EE frame wrt nominal end-effector frame (column major)
        :rtype: [bool, str]
        :return: [success status of service request, error msg if any]
        """
        retval = franka_interface.ArmInterface.set_EE_frame(self, frame)
        if retval:
            self.set_up_kinematics_interface()
        return retval

    def set_EE_at_frame(self, frame_name, timeout=5.0):
        """
        .. note:: This method is not available in simulated environment (when using PandaSimulator).

        Set new EE frame to the same frame as the link frame given by 'frame_name'.
        Motion controllers are stopped and restarted for switching. Also resets the 
        kinematic chain for PyKDL IK/FK computations.

        :type frame_name: str 
        :param frame_name: desired tf frame name in the tf tree
        :rtype: [bool, str]
        :return: [success status of service request, error msg if any]
        """
        retval = franka_interface.ArmInterface.set_EE_at_frame(self, frame_name, timeout)
        if retval:
            self.set_up_kinematics_interface()
        return retval

    def reset_EE_frame(self):
        """
        .. note:: This method is not available in simulated environment (when using PandaSimulator).

        Reset EE frame to default. (defined by 
        FrankaFramesInterface.DEFAULT_TRANSFORMATIONS.EE_FRAME 
        global variable defined in :py:class:`franka_tools.FrankaFramesInterface` 
        source code). 
        
        By default, this resets to align EE with the nominal-end effector
        frame (F_T_NE) in the flange frame (defined in Desk GUI).
        Motion controllers are stopped and restarted for switching. Also resets the 
        kinematic chain accordingly for PyKDL IK/FK computations.

        :rtype: [bool, str]
        :return: [success status of service request, error msg if any]
        """
        retval = franka_interface.ArmInterface.reset_EE_frame(self)
        if retval:
            self.set_up_kinematics_interface()
        return retval
    
    def _configure_gripper(self, gripper_joint_names):
        self._gripper = franka_interface.GripperInterface(
            ns=self._ns, gripper_joint_names=gripper_joint_names)
        if not self._gripper.exists:
            self._gripper = None
            return

    def get_gripper(self):
        """
            :return: gripper instance
            :rtype: franka_interface.GripperInterface

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
        Move to neutral pose (using trajectory controller, or moveit (if moveit is available))
        """
        self.move_to_neutral()

    def gripper_state(self):
        """
        Return Gripper state {'position', 'force'}. Only available if Franka gripper is connected.

        :rtype: dict ({str : numpy.ndarray (shape:(2,)), str : numpy.ndarray (shape:(2,))})
        :return: dict of position and force

          - 'position': :py:obj:`numpy.ndarray`
          - 'force': :py:obj:`numpy.ndarray`
        """
        gripper_state = {}

        if self._gripper:
            gripper_state['position'] = self._gripper.joint_ordered_positions()
            gripper_state['force'] = self._gripper.joint_ordered_efforts()

        return gripper_state

    def set_gripper_speed(self, speed):
        """
            Set velocity for gripper motion

            :param speed: speed ratio to set
            :type speed: float
        """
        if self._gripper:
            self._gripper.set_velocity(speed)

    def _update_tip_state(self, tipstate_msg):
        tip_state = {}

        time = tipstate_msg.timestamp
        tip_state['position'] = tipstate_msg.pose['position']
        ori = tipstate_msg.pose['orientation']
        force = tipstate_msg.effort['force']
        torque = tipstate_msg.effort['torque']
        tip_state['force_K'] = -tipstate_msg.effort_in_K_frame['force']
        tip_state['torque_K'] = -tipstate_msg.effort_in_K_frame['torque']

        tip_state['orientation'] = np.asarray([ori.w, ori.x, ori.y, ori.z])
        tip_state['linear_vel'] = tipstate_msg.velocity['linear']
        tip_state['angular_vel'] = tipstate_msg.velocity['angular']
        tip_state['time'] = {'secs': time.secs, 'nsecs': time.nsecs}

        tip_state['force'] = np.asarray([-force[0], -force[1], -force[2]])
        tip_state['torque'] = np.asarray(
            [-torque[0], -torque[1], -torque[2]])

        self._tip_state = copy.deepcopy(tip_state)

    def _update_state(self):

        now = rospy.Time.now()

        state = {}
        state['position'] = self.angles()
        state['velocity'] = self.velocities()
        state['effort'] = self.efforts()
        state['jacobian'] = self.jacobian(None)
        state['inertia'] = self.inertia(None)
        state['tip_state'] = self.tip_state()
        state['coriolis'] = self.coriolis_comp()
        state['gravity'] = self.gravity_comp()

        state['timestamp'] = {'secs': now.secs, 'nsecs': now.nsecs}
        state['ee_point'], state['ee_ori'] = self.ee_pose()

        tmp = state['jacobian'].dot(state['velocity'])

        state['ee_vel'], state['ee_omg'] = tmp[:3], tmp[3:]

        state['ft_reading'] = [self._tip_state
                               ['force'], self._tip_state['torque']]

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

        all_angles = [joint_angles[n] for n in joint_names]

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

        all_velocities = [joint_velocities[n] for n in joint_names]

        if include_gripper and self._gripper:
            all_velocities += self._gripper.joint_ordered_velocities()

        return np.array(all_velocities)

    def efforts(self, include_gripper=False):
        """
        :return: current joint efforts (measured torques)
        :rtype: [float]

        :param include_gripper: if True, append gripper joint efforts to list
        :type include_gripper: bool
        """
        joint_efforts = self.joint_efforts()

        joint_names = self.joint_names()

        all_efforts = [joint_efforts[n] for n in joint_names]

        if include_gripper and self._gripper:
            all_efforts += self._gripper.joint_ordered_efforts()

        return np.array(all_efforts)


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
        :return: tip (end-effector frame) state dictionary with keys 
            ['position', 'orientation', 'force', 'torque', 'force_K',
            'torque_K', 'linear_vel', 'angular_vel']. All are :py:obj:`numpy.ndarray` 
            objects of appropriate dims. 'force' and 'torque' are in the robot's base
            frame, while 'force_K' and 'torque_K' are in the stiffness frame.
        :rtype: dict {str: obj}
        """
        return self._tip_state

    def set_arm_speed(self, speed):
        """
        Set joint position speed (only effective for :py:meth:`move_to_joint_position`, 
        :py:meth:`move_to_joint_pos_delta`, and 
        :py:meth:`move_to_cartesian_pose <franka_interface.ArmInterface.move_to_cartesian_pose>`)

        :type speed: float
        :param speed: ratio of maximum joint speed for execution; range = [0.0,1.0]
        """
        self.set_joint_position_speed(speed)

    def _on_joint_states(self, msg):
        # Parent callback function is overriden to update robot state of this class

        franka_interface.ArmInterface._on_joint_states(self, msg)

        if self._arm_configured:
            self._update_tip_state(self.tip_states())
            self._state = self._update_state()
            self._on_state_callback(self._state)

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

        :param pos: desired width [m]
        :param force: desired force to be applied on object [N]
        :type pos: float
        :type force: float

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        if self._gripper is None:
            return

        width = min(self._gripper.MAX_WIDTH, max(self._gripper.MIN_WIDTH, pos))

        if force:
            holding_force = min(
                max(self._gripper.MIN_FORCE, force), self._gripper.MAX_FORCE)

            return self._gripper.grasp(width=width, force=holding_force)

        else:
            return self._gripper.move_joints(width)

    def exec_gripper_cmd_delta(self, pos_delta, force_delta=None):
        raise NotImplementedError(
            "PandaArm: 'exec_gripper_cmd_delta' not implemented")

    def exec_position_cmd(self, cmd):
        """
        Execute position control on the robot (raw positions). Be careful while using. Send smooth
        commands (positions that are very small distance apart from current position).

        :param cmd: desired joint postions, ordered from joint1 to joint7
                        (optionally, give desired gripper width as 8th element of list)
        :type cmd: [float]
        """

        if len(cmd) > 7:
            gripper_cmd = cmd[7:]
            self.exec_gripper_cmd(*gripper_cmd)

        joint_command = dict(zip(self.joint_names(), cmd[:7]))

        self.set_joint_positions(joint_command)

    def exec_position_cmd_delta(self, cmd):
        """
        Execute position control based on desired change in joint positions wrt current joint positions.

        :param cmd: desired joint postion changes, ordered from joint1 to joint7
        :type cmd: [float]
        """
        curr_q = self.joint_angles()
        joint_names = self.joint_names()

        joint_command = dict([(joint, curr_q[joint] + cmd[i])
                              for i, joint in enumerate(joint_names)])
        self.set_joint_positions(joint_command)

    def move_to_joint_pos_delta(self, cmd):
        """
        Execute motion (using moveit; if moveit not available attempts with trajectory controller) 
        based on desired change in joint position wrt to current joint positions

        :param cmd: desired joint postion changes, ordered from joint1 to joint7
        :type cmd: [float]
        """
        curr_q = self.joint_angles()
        joint_names = self.joint_names()

        joint_command = dict([(joint, curr_q[joint] + cmd[i])
                              for i, joint in enumerate(joint_names)])

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

    def ee_pose(self):
        """
        :return: end-effector pose as position and quaternion in global frame obtained directly from robot state
        :rtype: numpy.ndarray (pose), np.quaternion (orientation)
        """
        ee_point = np.asarray(self.endpoint_pose()['position'])

        ee_ori = self.endpoint_pose()['orientation']
        ee_ori = np.quaternion(ee_ori.w, ee_ori.x, ee_ori.y, ee_ori.z)

        return ee_point, ee_ori

    def ee_velocity(self, real_robot=True):
        """
        :return: end effector velocity (linear and angular) computed using finite difference
        :rtype: numpy.ndarray, numpy.ndarray

        :param real_robot: if False, computes ee velocity using finite difference
        :type real_robot: bool

        If real_robot is False, this is a simple finite difference based velocity
        computation. Please note that this might produce a bug since some values gets
        updated only if ee_velocity() is called. [TODO: This can be fixed by moving the update
        to the state callback method.]
        """

        if real_robot:

            ee_vel = self.endpoint_velocity()['linear']
            ee_omg = self.endpoint_velocity()['angular']

        else:

            time_now_new = time_in_seconds()

            ee_pos_new, ee_ori_new = self.ee_pose()

            dt = time_now_new - self._time_now_old

            ee_vel = (ee_pos_new - self._ee_pos_old) / dt

            ee_omg = compute_omg(ee_ori_new, self._ee_ori_old) / dt

            self._goal_ori_old = ee_ori_new
            self._goal_pos_old = ee_pos_new
            self._time_now_old = time_now_new

        return ee_vel, ee_omg
    
    def move_to_joint_position(self, joint_angles, timeout=10.0, threshold=0.00085, test=None, use_moveit=True):
        """
        Move to joint position specified (using MoveIt by default; if MoveIt server is not running then attempts with trajectory action client).

        .. note:: This method stops the currently active controller for trajectory tracking (and automatically restarts the controller(s) after execution of trajectory).

        :param joint_angles: desired joint positions, ordered from joint1 to joint7
        :type joint_angles: [float]
        :type timeout: float
        :param timeout: seconds to wait for move to finish [15]
        :type threshold: float
        :param threshold: position threshold in radians across each joint when
         move is considered successful [0.00085]
        :param test: optional function returning True if motion must be aborted
        :type use_moveit: bool
        :param use_moveit: if set to True, and movegroup interface is available, 
         move to the joint positions using moveit planner.
        """
        self.move_to_joint_positions(
            dict(zip(self.joint_names(), joint_angles)), timeout=timeout, threshold=threshold, test=test, use_moveit=use_moveit)

    def forward_kinematics(self, joint_angles=None, ori_type='quat'):
        """
        :return: position and orientaion of end-effector for the current/provided joint angles
        :rtype: [numpy.ndarray, numpy.ndarray (or) quaternion.quaternion]

        :param joint_angles: joint angles (optional) for which the ee pose is to be computed
        :type joint_angles: [float]
        :param ori_type: to specify the orientation representation to return ('quat','mat','eul')
        :type ori_type: str
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

        return position, rotation

    def cartesian_velocity(self, joint_angles=None):
        """
        Get cartesian end-effector velocity. To get velocity from franka_ros_interface directly, use method
        :py:func:`ee_velocity`.

        :return: end-effector velocity computed using kdl
        :rtype: numpy.ndarray

        :param joint_angles: joint angles (optional) 
        :type joint_angles: [float]

        """
        if joint_angles is None:
            argument = None
        else:
            argument = dict(zip(self.joint_names(), joint_angles))

        # only velocity
        return np.array(self._kinematics.forward_velocity_kinematics(argument))[0:3]

    def jacobian(self, joint_angles=None):
        """
        :return: jacobian matrix of robot at current state as computed using KDL (should match the value provided by libfranka through :py:meth:`franka_interface.ArmInterface.zero_jacobian` when no argument is passed)
        :rtype: numpy.ndarray

        :param joint_angles: joint angles (optional) for which the jacobian is to be computed
        :type joint_angles: [float]
        """
        if joint_angles is None:
            argument = None
        else:
            argument = dict(zip(self.joint_names(), joint_angles))

        jacobian = np.array(self._kinematics.jacobian(argument))

        return jacobian

    def inertia(self, joint_angles=None):
        """
        :return: inertia matrix of robot at current state as computed using KDL (should be close to the value provided by libfranka through :py:meth:`franka_interface.ArmInterface.joint_inertia_matrix` when no argument is passed; Exact match may not be available due to dynamics model and computation errors.)
        :rtype: numpy.ndarray

        :param joint_angles: joint angles (optional)
        :type joint_angles: [float]
        """
        if joint_angles is None:
            argument = None
        else:
            argument = dict(zip(self.joint_names(), joint_angles))

        return np.array(self._kinematics.inertia(argument))

    def inverse_kinematics(self, pos, ori=None, seed=None, null_space_goal=None, **kwargs):
        """
        :return: get the joint positions using inverse kinematics from the provided end-effector pose
        :rtype: bool (success), [float]

        :param pos: end-effector position (x,y,z)
        :type pos: [float]
        :param ori: end-effector orientation (quaternion)
        :type ori: [float] or np.quaternion
        :param seed: seed joints to start ik computation
        :type seed: [float]
        :param null_space_goal: null-space joint position if required
        :type null_space_goal: [float]

        kwargs are to avoid breaking of sister classes for arguments that are not used in this class.
        """
        success = False
        soln = None

        if ori is not None:
            # expects a pykdl quaternion which is of type x,y,z,w
            if isinstance(ori, np.quaternion):
                ori = np.array([ori.x, ori.y, ori.z, ori.w])

        soln = self._kinematics.inverse_kinematics(
            position=pos, orientation=ori, seed=seed)

        if soln is not None:
            success = True

        return success, soln


def main():
    rospy.init_node("panda_arm_untuck")

    arm = PandaArm(reset_frames=False)

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


if __name__ == '__main__':
    main()
