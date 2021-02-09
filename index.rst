Panda Robot
===========

A Python interface package built over the `Franka ROS Interface <franka_ros_interface_>`_ package for controlling and handling the Franka Emika Panda robot. Also works directly with `Panda Simulator <panda_simulator_>`_.

*NOTE: This version requires Franka ROS Interface v0.7.1. For usage with older versions, use Panda Robot branch 'v0.6.0' from Github.*

The package provides an extensive and unified API for controlling and managing the Franka Emika Robot (and gripper) using pre-defined low-level controllers (position, velocity, torque, joint impedance), MoveIt planners, and JointTrajectory action service.

**Features**

- Provides simple-intuitive interface classes with methods to directly and easily control the robot using low-level controllers, MoveIt planners, or Trajectory action client.
- Get real-time robot state, joint state, controller state, kinematics, dynamics, etc.
- Provides Kinematics computation (using `kdl`_).
- Integrated with gripper control.
- Manage frames transformation and controller switching using simple utility functions
- Works directly on simulated robot when using `Panda Simulator <panda_simulator_>`_ 
  providing direct sim-to-real and real-to-sim code transfer.

Go to `Project Source Code`_.

Installation
============

- Install `Franka ROS Interface <https://github.com/justagist/franka_ros_interface>`_ package. *This package should be installed from source (v0.7.1-dev or master branch) following all instructions in the* `Installation <https://github.com/justagist/franka_ros_interface#installation>`_ *section.*

- Clone the PandaRobot package to `src` folder of your catkin workspace. In catkin root run:

.. code-block:: bash

    $ catkin build
    $ source devel/setup.bash

Basic Usage
===========

**Note: The franka_ros_interface 'driver' should be running in the 'master' environment in one terminal (See** `Franka ROS Interface instructions <https://github.com/justagist/franka_ros_interface#usage>`_ **for details). Then, any code which uses PandaRobot or Franka ROS Interface should be run in 'master' or 'remote' environment (as appropriate).**

Basic usage of PandaRobot API is shown below. See `scripts`_ folder to see usage of some of the available methods, and example for real-time low-level control of the robot.

.. code-block:: python

    import rospy
    from aml_robot.panda_robot import PandaArm

    if __name__ == '__main__':
        rospy.init_node("panda_demo") # initialise ros node

        r = PandaArm() # create PandaArm instance

        r.move_to_neutral() # moves robot to neutral pose; uses moveit if available, else JointTrajectory action client

        pos,ori = r.ee_pose() # get current end-effector pose (3d position and orientation quaternion of end-effector frame in base frame)

        r.get_gripper().home_joints() # homes gripper joints
        r.get_gripper().open() # open gripper

        r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01]) # move robot to the specified pose

        raw_input("Hit enter to continue") # hit enter to continue

        r.move_to_cartesian_pose(pos,ori) # move the robot end-effector to pose specified by 'pos','ori'

.. toctree::
   :caption: Contents:
   :hidden:

   self
   DOC

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Go to `Project Source Code`_.

**LICENSE:**

|License|

Copyright (c) 2019-2021, Saif Sidhik

.. _panda_simulator: https://github.com/justagist/panda_simulator
.. _KDL library: http://wiki.ros.org/kdl
.. _franka_panda_description: https://github.com/justagist/franka_panda_description
.. _franka_ros_interface: https://github.com/justagist/franka_ros_interface
.. _this paper: https://hal.inria.fr/hal-02265293/document

.. _Python Documentation: https://justagist.github.io/franka_ros_interface

.. _FCI documentation: https://frankaemika.github.io/docs/installation_linux.html
.. _franka_panda_description: https://github.com/justagist/franka_panda_description
.. _Related Packages: #related-packages
.. _Environments: #the-frankash-environments
.. _install from source: https://frankaemika.github.io/docs/installation_linux.html#building-from-source
.. _kdl: http://wiki.ros.org/kdl

.. _Python API Documentation: https://justagist.github.io/panda_robot
.. _Project Source Code: https://github.com/justagist/panda_robot

.. _scripts: https://github.com/justagist/panda_robot/tree/master/scripts


.. |License| image:: https://img.shields.io/badge/License-Apache2.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
