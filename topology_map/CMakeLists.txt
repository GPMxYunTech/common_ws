SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
cmake_minimum_required(VERSION 2.8.3)
project(topology_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  tf
  topic_tools
  move_base_msgs
  visualization_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate actions in the 'action' folder
#add_action_files(
#  DIRECTORY action FILES check_msg.action
#   FILES
#   Action1.action
#   Action2.action
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES actionlib_msgs std_msgs
#   geometry_msgs#   sensor_msgs#   std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dashgo_tools
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs serial std_msgs tf topic_tools
  laser_line_extraction
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
 include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(dashgo_tools
#   src/${PROJECT_NAME}/dashgo_tools.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dashgo_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(topology_map node/topology_map.cpp src/dijkstra.cpp )
# target_link_libraries(multi_goal_loop ${catkin_LIBRARIES})

add_executable(multi_goal_loop src/multi_goal_loop.cpp )
target_link_libraries(multi_goal_loop ${catkin_LIBRARIES})

add_executable(auto_goal_path src/auto_goal_path.cpp)
target_link_libraries(auto_goal_path ${catkin_LIBRARIES})

add_executable(multi_goal_loop_carrot src/multi_goal_loop_carrot.cpp)
target_link_libraries(multi_goal_loop_carrot ${catkin_LIBRARIES})

add_executable(auto_path src/auto_path.cpp)
target_link_libraries(auto_path ${catkin_LIBRARIES})

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})

add_executable(tricycle_nav 
src/tricycle_nav.cpp
src/pbPlots.cpp
src/supportLib.cpp
)
target_link_libraries(tricycle_nav ${catkin_LIBRARIES})

add_executable(obstacal_right_angle_advoid_node src/obstacal_right_angle_advoid_node)
target_link_libraries(obstacal_right_angle_advoid_node ${catkin_LIBRARIES})
#add_executable(auto_goal_path src/auto_goal_path.cpp)
#target_link_libraries(auto_goal_path ${catkin_LIBRARIES})

#0429 char add end
#add_executable(barcodemove_server src/barcodemove_server.cpp)
#target_link_libraries(barcodemove_server ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(dashgo_tools_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(dashgo_tools_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dashgo_tools dashgo_tools_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dashgo_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
