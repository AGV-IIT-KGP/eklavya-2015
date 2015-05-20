cmake_minimum_required(VERSION 2.8.3)
project(dummy_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)
catkin_package()

#(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(imu_to_yaw_publisher src/yaw_publisher.cpp)
target_link_libraries(imu_to_yaw_publisher ${catkin_LIBRARIES})
add_dependencies(imu_to_yaw_publisher imu_to_yaw_publisher_generate_messages_cpp)
