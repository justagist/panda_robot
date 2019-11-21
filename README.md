# Panda Robot (ROS Python)

A Python interface package built over the [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) package for controlling and handling the Franka Emika Panda robot. Also works directly with [*panda_simulator*](https://github.com/justagist/panda_simulator).

#### Dependency
- [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) package

### Features
- Provides simple-intuitive interface classes with methods to directly and easily control the robot.
- Get real-time robot state, joint state, controller state, kinematics, dynamics, etc.
- Provides Kinematics computation (using [KDL library](http://wiki.ros.org/kdl)).
- Integrated with gripper control.
- Manage frames transformation and controller switching using simple utility functions
- Works directly on simulated robot when using [*panda_simulator*](https://github.com/justagist/panda_simulator) providing direct sim-to-real and real-to-sim code transfer.

See example scripts for usage.
**Detailed documentation in source files.**