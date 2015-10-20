cmake_minimum_required(VERSION 2.8.3)
project(eklavya_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_plugins gazebo_ros gazebo_msgs roscpp std_msgs)
find_package(gazebo)
catkin_package(
)

execute_process (COMMAND ${PROJECT_SOURCE_DIR}/setbashrc.bash)

install(
  DIRECTORY launch worlds Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
add_executable(encoder_publisher src/encoder_publisher.cpp)
target_link_libraries( encoder_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
