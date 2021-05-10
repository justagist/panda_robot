cmake_minimum_required(VERSION 2.8.3)
project(panda_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  franka_interface
  franka_core_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package()

catkin_python_setup()
catkin_install_python(PROGRAMS src/panda_robot/panda_robot.py 
                               demos/demo_endeffector_keyboard.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


