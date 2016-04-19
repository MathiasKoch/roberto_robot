cmake_minimum_required(VERSION 2.8.3)
project(roberto_msgs)


find_package(catkin REQUIRED
	std_msgs 
	genmsg
	message_generation)


add_message_files(DIRECTORY msg FILES 
	MotorState.msg
	JointCommand.msg
	Line.msg)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS
	message_runtime 
	std_msgs
)


