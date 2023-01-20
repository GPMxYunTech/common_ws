################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_automatic_parking_vision)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  apriltag_ros
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs apriltag_ros)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################

#catkin_install_python(PROGRAMS 
#  nodes/automatic_parking_main
# 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
catkin_install_python(PROGRAMS 
  nodes/automatic_parking_vision_task
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



catkin_install_python(PROGRAMS 
  nodes/navigation_to_pbvs
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS 
  nodes/test
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS 
  nodes/automatic_parking_vision
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS 
  nodes/lbvs
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
