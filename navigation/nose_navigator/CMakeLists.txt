cmake_minimum_required(VERSION 2.8.3)
project(nose_navigator)

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


add_executable(nose_navigator src/nose_navigator.cpp)
target_link_libraries(nose_navigator ${catkin_LIBRARIES})
add_dependencies(nose_navigator nose_navigator_generate_messages_cpp)