cmake_minimum_required(VERSION 2.8.3)
project(eklavya4_roboteq)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++0x)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SetSpeed.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES eklavya4_roboteq
  CATKIN_DEPENDS roscpp std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)



## Declare a cpp executable
 add_executable(roboteq_server src/roboteq_server.cpp src/RoboteqDevice.cpp)
 add_executable(roboteq_client src/command_subscriber_roboteq_client.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(roboteq_server
   ${catkin_LIBRARIES}
 )
 target_link_libraries(roboteq_client
   ${catkin_LIBRARIES}
 )
