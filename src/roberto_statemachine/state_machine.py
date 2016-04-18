#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import random
from std_msgs.msg import String

import threading

robot = [0, 0, 0] #x,y,rot
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = None

DEFAULT_TIMEOUT = 5

# BASIC CLASSES TO SUBCLASS

# Check steps are fast checks for checking wether an action is completed
class Check(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[True, False])

# Action steps create commands for the motors and are limited to the current rate
class Action(smach.State):
	def __init__(self, timeout = DEFAULT_TIMEOUT):
		smach.State.__init__(self, outcomes=['confirm','timeout'])

		self._exec_counter = -1
		self._exec_timeout = rospy.Duration.from_sec(timeout)
		self._initial_time = None

	def execute(self, userdata):
		# initial run
		if not self._initial_time:
			self._initial_time = rospy.Time.now()
			self._exec_counter += 1
		
		# sleep to rate
		rate.sleep()

		# check timeout
		if rospy.Time.now() - self._initial_time > self._exec_timeout:
			return "timeout"

		return self.executeAction(userdata)

	def executeAction(self, userdata):
		return NotImplementedError



# ACTION STEPS
class Drive(Action):
	def __init__(self, speed = 0.5, timeout = DEFAULT_TIMEOUT):
		super(Drive, self).__init__(timeout = timeout)
		self.speed = speed

	def executeAction(self, userdata):
		robot[0] += self.speed
		pub.publish(str(self.speed))
		return 'confirm'

class Turn(Action):
	def __init__(self, speed = 0.1, timeout = DEFAULT_TIMEOUT):
		super(Turn, self).__init__(timeout = timeout)
		self.speed = speed

	def executeAction(self, userdata):
		rospy.loginfo('send turn command')
		robot[2] += self.speed;
		pub.publish(str(self.speed))
		return 'confirm'


# CHECK STEPS
class CheckCrossing(Check):
    def execute(self, userdata):
		rospy.loginfo("checkCrossing @ " + str(robot));
		return False
		return robot[0] == 5

class CheckWaitFor(Check):
    def execute(self, userdata):
    	rospy.loginfo("checkWaitFor @ " + str(robot));
    	return False
    	return random.randint(0,10)>5

class CheckLine(Check):
    def execute(self, userdata):
    	rospy.loginfo("checkLine @ " + str(robot));
    	return False
    	return random.randint(0,10)>5



# OTHER
class RateChange(smach.State):
	def __init__(self, rate):
		smach.State.__init__(self, outcomes=['done'])
		self.new_rate = rate

	def execute(self, userdata):
		global rate
		rate = rospy.Rate(self.new_rate)
		return 'done'



# PREMADE State Machines
def createlineSM():
    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('Drive', Drive(speed=0.25, timeout=15), 
                               transitions={'confirm':'CheckCrossing', 'timeout':'done'})
        smach.StateMachine.add('CheckCrossing', CheckCrossing(), 
                               transitions={True:'done', False:"CheckWaitFor"})
        smach.StateMachine.add('CheckWaitFor', CheckWaitFor(), 
                               transitions={True:'done', False:"Drive"})
    return sm

def createlineSM2():
    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        # Add states to the container
        smach.StateMachine.add('Turn', Turn(timeout = 50), 
                               transitions={'confirm':'CheckCrossing', 'timeout':'done'})
        smach.StateMachine.add('CheckCrossing', CheckCrossing(), 
                               transitions={True:'done', False:"CheckWaitFor"})
        smach.StateMachine.add('CheckWaitFor', CheckWaitFor(), 
                               transitions={True:'done', False:"Turn"})
    return sm



def main():
    rospy.init_node('smach_example_state_machine')
    global rate
    rate = rospy.Rate(10)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:
        smach.StateMachine.add('SUB', createlineSM(),
        						transitions={'done':'RateChange'})
        smach.StateMachine.add('RateChange', RateChange(3),
        						transitions={'done':'SUB2'})
        smach.StateMachine.add('SUB2', createlineSM2(),
        						transitions={'done':'done'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
