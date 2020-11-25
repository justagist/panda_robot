# Panda Robot (ROS / Python 2) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747459.svg)](https://doi.org/10.5281/zenodo.3747459)

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/104807d6e9d74377ac40c827d9d261e3)](https://www.codacy.com/manual/justagist/panda_robot?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=justagist/panda_robot&amp;utm_campaign=Badge_Grade) [![franka_ros_interface_version](https://img.shields.io/badge/franka_ros_interface-v0.6.0,%20v0.7.1-yellow.svg)](https://github.com/justagist/franka_ros_interface)

A Python interface package built over the [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) package, combining its different classes to provide a unified interface for controlling and handling the Franka Emika Panda robot. Also works directly with [*panda_simulator*](https://github.com/justagist/panda_simulator).

The package provides an extensive and unified [API](https://justagist.github.io/panda_robot/) for controlling and managing the Franka Emika Robot (and gripper) using pre-defined low-level controllers (position, velocity, torque, joint impedance), MoveIt planners, and JointTrajectory action service.

### Features

- Provides simple-intuitive interface classes with methods to directly and easily control the robot using low-level controllers, MoveIt planners, or Trajectory action client.
- Get real-time robot state, joint state, controller state, kinematics, dynamics, etc.
- Provides Kinematics computation (using [KDL library](http://wiki.ros.org/kdl)).
- Integrated with gripper control.
- Manage frames transformation and controller switching using simple utility functions.
- Works directly on simulated robot when using [*panda_simulator*](https://github.com/justagist/panda_simulator) providing direct sim-to-real and real-to-sim code transfer.

**DOCUMENTATION**: https://justagist.github.io/panda_robot/

  ![vid](assets/panda_robot_demo.gif)
 Watch video [here](https://youtu.be/4bEVysUIvOY)

  ![vid](assets/panda_simulator.gif)
 Watch video [here](https://www.youtube.com/watch?v=NdSbXC0r7tU)


#### Dependency
- [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) package

### Installation

Clone package to `src` folder of your catkin workspace. In catkin root run:

```sh
 catkin build
 source devel/setup.bash
```

### Usage

Example: Testing interface in terminal

```bash
>> python # start interactive python session; make sure the correct ros workspace is sourced.
>> import rospy
>> from panda_robot import PandaArm
>> rospy.init_node("panda_demo") # initialise ros node

>> r = PandaArm() # create PandaArm instance

>> r.move_to_neutral() # moves robot to neutral pose; uses moveit if available, else JointTrajectory action client

>> r.get_gripper().home_joints() # homes gripper joints
>> r.get_gripper().open() # open gripper

>> r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01]) # move robot to the specified pose
```

See script (`scripts/controller_test.py`) to see how the robot can be controlled using low-level joint controllers.

See script (`scripts/env.py`), and run it interactively (`python -i env.py`) for testing other available functionalities. Other available functionalities: https://justagist.github.io/panda_robot/

### License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2020, Saif Sidhik

If you use this software, please cite it using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747413.svg)](https://doi.org/10.5281/zenodo.3747413).
