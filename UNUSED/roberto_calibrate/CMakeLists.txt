cmake_minimum_required(VERSION 2.8.3)
project(roberto_calibrate)

find_package(catkin 
	REQUIRED COMPONENTS
	roscpp
	roslib
	roberto_msgs
	roberto_i2c)



find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

catkin_package(
	CATKIN_DEPENDS 
	roberto_msgs
	roberto_i2c
	roslib
	roscpp
	DEPENDS yaml-cpp
	)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include
                    ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} 
	src/i2c_calib.cpp
	src/RTIMUMagCal.cpp
	src/RTIMUAccelCal.cpp
	src/RTMath.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      yaml-cpp)


install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})