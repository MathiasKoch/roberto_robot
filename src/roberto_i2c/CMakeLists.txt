cmake_minimum_required(VERSION 2.8.3)
project(roberto_i2c)

find_package(catkin 
	REQUIRED COMPONENTS
	roscpp
	roberto_msgs)


catkin_package(
	CATKIN_DEPENDS 
	roberto_msgs
	roscpp
	DEPENDS 
	Boost)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} 
	src/i2c_node.cpp
	src/gpio.cpp
	src/I2CBus.cpp
	src/OLED.cpp
	src/OLED_GFX.cpp
	src/RTFusionRTQF.cpp
	src/RTIMU.cpp
	src/RTIMUMPU9250.cpp
	src/RTIMUSettings.cpp
	src/RTMath.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})