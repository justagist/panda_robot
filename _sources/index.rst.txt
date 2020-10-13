
Panda Robot
===========

A Python interface package built over the `franka_ros_interface`_ package for controlling and handling the Franka Emika Panda robot. Also works directly with `panda_simulator`_.

**Features**

- Provides simple-intuitive interface classes with methods to directly and easily control the robot.
- Get real-time robot state, joint state, controller state, kinematics, dynamics, etc.
- Provides Kinematics computation (using `kdl`_).
- Integrated with gripper control.
- Manage frames transformation and controller switching using simple utility functions
- Works directly on simulated robot when using `panda_simulator`_ 
  providing direct sim-to-real and real-to-sim code transfer.

Go to `Project Source Code`_.

.. toctree::
   :maxdepth: 4
   :caption: Contents:
   :numbered:

   usage_demo
   DOC

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Go to `Project Source Code`_.

**LICENSE:**

|License|

Copyright (c) 2019-2020, Saif Sidhik

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


.. |License| image:: https://img.shields.io/badge/License-Apache2.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0

