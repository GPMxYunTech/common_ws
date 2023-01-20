cmake_minimum_required(VERSION 3.0.2)
project(car_controller)



find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial tf nav_msgs)




generate_messages(DEPENDENCIES std_msgs)
## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
include 
${catkin_INCLUDE_DIRS} 
/home/ericlai/forklift_ws/src/car_controller/include/hello.h)

add_executable(main src/main.cpp src/hello.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main car_controller_generate_messages_cpp)


include_directories(
include 
${catkin_INCLUDE_DIRS} 
/home/ericlai/forklift_ws/src/car_controller/include/stm32_controller.h)

add_executable(ros_communication src/ros_communication.cpp src/stm32_controller.cpp)
target_link_libraries(ros_communication ${catkin_LIBRARIES})
add_dependencies(ros_communication car_controller_generate_messages_cpp)

#add_executable(stm32_controller src/stm32_controller.cpp)
#target_link_libraries(stm32_controller ${catkin_LIBRARIES})
#add_dependencies(stm32_controller car_controller_generate_messages_cpp)

add_executable(car_controller src/car_controller.cpp)
target_link_libraries(car_controller ${catkin_LIBRARIES})
add_dependencies(car_controller car_controller_generate_messages_cpp)

add_executable(car_controller_pid src/car_controller_pid.cpp)
target_link_libraries(car_controller_pid ${catkin_LIBRARIES})
add_dependencies(car_controller_pid car_controller_generate_messages_cpp)

add_executable(car_listener src/car_listener.cpp)
target_link_libraries(car_listener ${catkin_LIBRARIES})
add_dependencies(car_listener car_controller_generate_messages_cpp)

add_executable(car_driver src/car_driver.cpp)
target_link_libraries(car_driver ${catkin_LIBRARIES})
add_dependencies(car_driver car_controller_generate_messages_cpp)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
