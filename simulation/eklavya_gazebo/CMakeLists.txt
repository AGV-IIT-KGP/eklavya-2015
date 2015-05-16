cmake_minimum_required(VERSION 2.8.3)
project(eklavya_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
)

execute_process (COMMAND ${PROJECT_SOURCE_DIR}/setbashrc.bash)

install(
  DIRECTORY launch worlds Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

