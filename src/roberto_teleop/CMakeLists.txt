cmake_minimum_required(VERSION 2.8.3)
project(roberto_teleop)

## Find catkin macros and libraries
find_package(catkin 
	REQUIRED COMPONENTS 
	roscpp 
	roberto_msgs 
	joy)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  roscpp 
  roberto_msgs 
  joy
  DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(roberto_teleop_joy
		src/teleop_joy.cpp)

target_link_libraries(roberto_teleop_joy 
	${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/roberto_teleop_key
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS roberto_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)