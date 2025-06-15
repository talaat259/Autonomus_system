cmake_minimum_required(VERSION 3.0.2)
project(camera_cone_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages






find_package(catkin REQUIRED  aam_common_msgs roscpp )

set(PROJECT_DEPS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  aam_common_msgs
  visualization_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_cone_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)




catkin_install_python(PROGRAMS src/camera_cone_detection.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


