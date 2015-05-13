cmake_minimum_required(VERSION 2.8.3)
project(high_level_planner_igvc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  decision_making
  decision_making_parser
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES high_level_planner_igvc
#  CATKIN_DEPENDS decision_making roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)
add_executable(fsm_high_level_planner_igvc src/fsm_high_level_planner_igvc.cpp src/strategy_planner.cpp)
add_executable(publish_nml src/publish_nml.cpp)
add_executable(publish_confidence src/publish_confidence.cpp)
add_executable(is_test_mode src/is_test_mode.cpp)
target_link_libraries(is_test_mode ${catkin_LIBRARIES})
target_link_libraries(fsm_high_level_planner_igvc ${catkin_LIBRARIES})
target_link_libraries(publish_nml ${catkin_LIBRARIES})
target_link_libraries(publish_confidence ${catkin_LIBRARIES})

decision_making_parsing(
	src/fsm_high_level_planner_igvc.cpp
)
install(TARGETS fsm_high_level_planner_igvc
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
