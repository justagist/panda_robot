# Panda Robot (ROS Python)

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/104807d6e9d74377ac40c827d9d261e3)](https://www.codacy.com/manual/justagist/panda_robot?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=justagist/panda_robot&amp;utm_campaign=Badge_Grade)

A Python interface package built over the [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) package for controlling and handling the Franka Emika Panda robot. Also works directly with [*panda_simulator*](https://github.com/justagist/panda_simulator).

### Features
- Provides simple-intuitive interface classes with methods to directly and easily control the robot.
- Get real-time robot state, joint state, controller state, kinematics, dynamics, etc.
- Provides Kinematics computation (using [KDL library](http://wiki.ros.org/kdl)).
- Integrated with gripper control.
- Manage frames transformation and controller switching using simple utility functions
- Works directly on simulated robot when using [*panda_simulator*](https://github.com/justagist/panda_simulator) providing direct sim-to-real and real-to-sim code transfer.

  ![vid](assets/panda_robot_demo.gif)
 Watch video [here](https://youtu.be/4bEVysUIvOY)

  ![vid](assets/panda_simulator.gif)
 Watch video [here](https://www.youtube.com/watch?v=NdSbXC0r7tU)

**DOCUMENTATION**: https://justagist.github.io/panda_robot/

#### Dependency
- [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) package

### Installation

Clone package to `src` folder of your catkin workspace. In catkin root run:

```sh
 catkin build
 source devel/setup.bash
```

See example scripts (`scripts/env.py`) for usage.

### License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2020, Saif Sidhik

If you use this software, please cite it using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747413.svg)](https://doi.org/10.5281/zenodo.3747413).
