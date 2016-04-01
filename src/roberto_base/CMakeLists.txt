cmake_minimum_required(VERSION 2.8.3)
project(roberto_base)

find_package(catkin 
	REQUIRED COMPONENTS
	rosserial_python
	roberto_msgs)


catkin_package(
	CATKIN_DEPENDS 
	roberto_msgs
	rosserial_python)

include_directories(include
                    ${catkin_INCLUDE_DIRS})


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)