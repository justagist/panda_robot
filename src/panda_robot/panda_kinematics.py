#!/usr/bin/python

import numpy as np
import PyKDL

import rospy
from copy import deepcopy

from franka_interface.utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

class panda_kinematics(object):
    """
    Franka Kinematics with PyKDL

    This will match with the RobotState if the tip frames are the same.
    
    """
    def __init__(self, limb, description = None):

        if description is None:
            self._franka = URDF.from_parameter_server(key='robot_description')
        else:
            self._franka = URDF.from_xml_file(description)

        self._kdl_tree = kdl_tree_from_urdf_model(self._franka)
        self._base_link = self._franka.get_root()
        self._tip_link = limb.name + '_hand' if limb.has_gripper else '_link8' # ---- hand frame does not work
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)

        self._limb_interface = limb
        self._joint_names = deepcopy(self._limb_interface.joint_names())
        # if self._limb_interface.has_gripper:
        #     self._joint_names += self._limb_interface.get_gripper().joint_names()
        self._num_jnts = len(self._joint_names)

        # KDL Solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain,
                                                   self._fk_p_kdl,
                                                   self._ik_v_kdl)
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._arm_chain)
        self._dyn_kdl = PyKDL.ChainDynParam(self._arm_chain,
                                            PyKDL.Vector.Zero())

    def print_robot_description(self):
        nf_joints = 0
        for j in self._franka.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print "URDF non-fixed joints: %d;" % nf_joints
        print "URDF total joints: %d" % len(self._franka.joints)
        print "URDF links: %d" % len(self._franka.links), [link.name for link in self._franka.links]
        print "KDL joints: %d" % self._kdl_tree.getNrOfJoints()
        print "KDL segments: %d" % self._kdl_tree.getNrOfSegments()

    def print_kdl_chain(self):
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            print '* ' + self._arm_chain.getSegment(idx).getName()

    def joints_to_kdl(self, type, values=None):
        kdl_array = PyKDL.JntArray(self._num_jnts)
        pos_array = PyKDL.JntArray(self._num_jnts)

        if values is None:
            if type == 'positions':
                cur_type_values = self._limb_interface.joint_angles()
                # if self._limb_interface.has_gripper:
                #     cur_type_values.update(self._limb_interface.get_gripper().joint_positions())
            elif type == 'velocities':
                cur_type_values = self._limb_interface.joint_velocities()
                pos_list = self._limb_interface.joint_angles()
                # if self._limb_interface.has_gripper:
                #     cur_type_values.update(self._limb_interface.get_gripper().joint_velocities())
                #     pos_list.update(self._limb_interface.get_gripper().joint_positions())

            elif type == 'torques':
                cur_type_values = self._limb_interface.joint_efforts()
                # if self._limb_interface.has_gripper:
                #     cur_type_values.update(self._limb_interface.get_gripper().joint_efforts())
        else:
            cur_type_values = values
        for idx, name in enumerate(self._joint_names):
            kdl_array[idx] = cur_type_values[name]
            if type == 'velocities':
                pos_array[idx] = pos_list[name]
        # print pos_list
        # print "THIS", cur_type_values
        if type == 'velocities':
            kdl_array = PyKDL.JntArrayVel(pos_array, kdl_array) # ----- using different constructor for getting velocity fk
        return kdl_array

    def kdl_to_mat(self, data):
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat

    def forward_position_kinematics(self,joint_values=None):
        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(self.joints_to_kdl('positions',joint_values),
                                 end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def forward_velocity_kinematics(self,joint_velocities=None):
        end_frame = PyKDL.FrameVel()
        self._fk_v_kdl.JntToCart(self.joints_to_kdl('velocities',joint_velocities),
                                 end_frame)
        # print 
        return end_frame.GetTwist()

    def inverse_kinematics(self, position, orientation=None, seed=None):
        ik = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation is not None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        if orientation is not None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_jnts)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            return result
        else:
            return None

    def jacobian(self,joint_values=None):
        jacobian = PyKDL.Jacobian(self._num_jnts)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self.kdl_to_mat(jacobian)

    def jacobian_transpose(self,joint_values=None):
        return self.jacobian(joint_values).T

    def jacobian_pseudo_inverse(self,joint_values=None):
        return np.linalg.pinv(self.jacobian(joint_values))


    def inertia(self,joint_values=None):
        inertia = PyKDL.JntSpaceInertiaMatrix(self._num_jnts)
        self._dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self.kdl_to_mat(inertia)

    def cart_inertia(self,joint_values=None):
        js_inertia = self.inertia(joint_values)
        jacobian = self.jacobian(joint_values)
        return np.linalg.inv(jacobian * np.linalg.inv(js_inertia) * jacobian.T)


if __name__ == '__main__':
    
    rospy.init_node('test')
    r = PandaArm()
    # print r.has_gripper
    kin = panda_kinematics(r)
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

    