Panda Robot |Python 2.7, 3.6+| |ROS Version|
============================================

|PyPI| |Codacy Badge|

A Python interface package built over the `Franka ROS Interface <franka_ros_interface_>`_ package for controlling and handling the Franka Emika Panda robot. Also works directly with `Panda Simulator <panda_simulator_>`_.

The package provides an extensive and unified `API <Python API Documentation_>`_ for controlling and
managing the Franka Emika Robot (and gripper) using pre-defined
low-level controllers (position, velocity, torque, joint impedance),
MoveIt planners, and JointTrajectory action service.

*NOTE: This version requires* `Franka ROS Interface
v0.7.1 <franka_ros_interface_>`_ *(‘master’/‘v0.7.1-dev’ branch) to be installed. For usage
with older versions, use Panda Robot branch* `v0.6.0`_ *from Github.*

**Features**

- Provides simple-intuitive interface classes with methods to directly and easily control the robot using low-level controllers, MoveIt planners, or Trajectory action client.
- Get real-time robot state, joint state, controller state, kinematics, dynamics, etc.
- Provides Kinematics computation (using `kdl`_).
- Integrated with gripper control.
- Manage frames transformation and controller switching using simple utility functions
- Works directly on simulated robot when using `Panda Simulator <panda_simulator_>`_ 
  providing direct sim-to-real and real-to-sim code transfer.

.. image:: https://raw.githubusercontent.com/justagist/franka_ros_interface/master/assets/panda_robot_demo.gif
   :target: https://youtu.be/4bEVysUIvOY

Go to `Project Source Code`_.


Installation
============

**NOTE:** This branch should work with ROS Melodic and ROS Noetic.
Tested on:

=========== =======================
ROS Version Required Python Version
=========== =======================
Melodic     2.7+
Noetic      3.6+
=========== =======================

**The following dependencies have to be met before installing
PandaRobot**:

-  Requires ROS Melodic or Noetic (preferably the ``desktop-full``
   version to cover all dependencies such as PyKDL and MoveIt)

-  `Franka ROS Interface <https://github.com/justagist/franka_ros_interface>`_ package. *This package should be installed
   from source (v0.7.1 or master branch) following all instructions in
   the*\ `Installation <https://github.com/justagist/franka_ros_interface#installation>`_\ *section. Installing this package correctly
   would also resolve all the other dependencies for PandaRobot.*

Once the dependencies are installed, the package can be installed either
from pypi, or by building from source. Note that the installation may be
successful even if the above dependencies are not met, but the package
cannot be used until the dependencies are installed.

Install with pip
~~~~~~~~~~~~~~~~

|PyPI|

.. code:: bash

   pip install panda-robot

**NOTE: This will not check for the required ROS dependencies. They have
to be installed as described in the previous section.**

Build from source
~~~~~~~~~~~~~~~~~

If you want to install the package from source, you can either clone
this repository and run ``python setup.py install``, or build it as a
catkin package in your ROS workspace. To build as catkin package:

-  Clone this repo to ``src`` folder of your catkin workspace.

-  In catkin workspace root, run:

.. code:: sh

    catkin build
    source devel/setup.bash

*Note: This package is written to be compatible with both Python 2 and
3, so make sure you have the Python* ``future`` *module installed*
(``pip install future``).

Basic Usage
===========

**Note: If using with a real physical Franka Emika Panda robot, the franka_ros_interface 'driver' should be running in the 'master' environment in one terminal (See** `Franka ROS Interface instructions <https://github.com/justagist/franka_ros_interface#usage>`_ **for details). Then, any code which uses PandaRobot or Franka ROS Interface should be run in 'master' or 'remote' environment (as appropriate). When using with** `Panda Simulator <https://github.com/justagist/panda_simulator>`_ **, this package can be used directly without the need for any specific environment as long as this package, the simulator package, and Franka ROS Interface packages are in the same ROS workspace, and correctly sourced.**

Basic usage of PandaRobot API is shown below. See `scripts`_ folder to see usage of some of the available methods, and example for real-time low-level control of the robot.

.. code-block:: python

    import rospy
    from panda_robot import PandaArm

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

If you use this software for research, please consider citing Franka ROS Interface using |doi|.

.. _panda_simulator: https://github.com/justagist/panda_simulator
.. _KDL library: http://wiki.ros.org/kdl
.. _franka_ros_interface: https://github.com/justagist/franka_ros_interface
.. _v0.6.0: https://github.com/justagist/panda_robot/tree/v0.6.0

.. _FCI documentation: https://frankaemika.github.io/docs/installation_linux.html
.. _Related Packages: #related-packages
.. _Environments: #the-frankash-environments
.. _install from source: https://frankaemika.github.io/docs/installation_linux.html#building-from-source
.. _kdl: http://wiki.ros.org/kdl

.. _Python API Documentation: https://justagist.github.io/panda_robot/DOC
.. _Project Source Code: https://github.com/justagist/panda_robot

.. _scripts: https://github.com/justagist/panda_robot/tree/master/scripts


.. |License| image:: https://img.shields.io/badge/License-Apache2.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
.. |ROS Version| image:: https://img.shields.io/badge/ROS-Melodic,%20Noetic-brightgreen.svg?logo=ros
   :target: https://ros.org/
.. |Python 2.7, 3.6+| image:: https://img.shields.io/badge/python-2.7,%203.6+-blue.svg?logo=python
   :target: https://www.python.org/downloads/release/python-360/
.. |doi| image:: https://zenodo.org/badge/199485892.svg
   :target: https://zenodo.org/badge/latestdoi/199485892
.. |PyPI| image:: https://img.shields.io/pypi/v/panda-robot?color=blue
   :target: https://pypi.org/project/panda-robot/
.. |Codacy Badge| image:: https://api.codacy.com/project/badge/Grade/104807d6e9d74377ac40c827d9d261e3
   :target: https://www.codacy.com/manual/justagist/panda_robot?utm_source=github.com&utm_medium=referral&utm_content=justagist/panda_robot&utm_campaign=Badge_Grade