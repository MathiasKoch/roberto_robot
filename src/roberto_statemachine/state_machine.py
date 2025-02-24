#!/usr/bin/env python

import roslib; roslib.load_manifest('roberto_statemachine')
import rospy
import smach
import smach_ros
import random
from std_msgs.msg import String
from roberto_msgs.msg import Line
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from roberto_msgs.msg import MotorState


import threading

#pub = rospy.Publisher('chatter', String, queue_size=10)
motor_pub = rospy.Publisher('cmd_vel', MotorState, queue_size=10)
line_pub = rospy.Publisher('linesensor_active', Float32, queue_size=10)
rate = None

DEFAULT_TIMEOUT = 15

linesensor_angle = 0
spin_done = False

followLineId = -1

joyTimeout = 0.5
joyTime = None
joyActive = False


def spin_callback(data):
	global spin_done
	#if data.data[3] <= 0:
	#	spin_done = True
	#else:
	spin_done = False

def joy_callback(data):
	global joyTime
	global joyActive
	joyTime = rospy.Time.now()
	joyActive = True
	motor_pub.publish(data)



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
		global joyTime
		global joyTimeout
		global joyActive
		# initial run
		if not self._initial_time:
			self._initial_time = rospy.Time.now()
			self._exec_counter += 1
		
		# sleep to rate
		rate.sleep()

		# check Joystick timeout on message
		if rospy.Time.now() - joyTime > rospy.Duration.from_sec(joyTimeout):
			joyActive = False

		# check timeout
		if rospy.Time.now() - self._initial_time > self._exec_timeout:
			return "timeout"

		#print "time: %u" % self._exec_timeout;

		return self.executeAction(userdata)

	def executeAction(self, userdata):
		return NotImplementedError



# ACTION STEPS
class Drive(Action):
	def __init__(self, speed = 0.5, timeout = DEFAULT_TIMEOUT):
		super(Drive, self).__init__(timeout = timeout)
		self.speed = speed

	def executeAction(self, userdata):
		#msg = MotorState()
		#msg.speed = self.speed
		#msg.heading_angle = 0
		#msg.mode = msg.DRIVE_MODE_PIVOT
		#print msg.heading_angle;
		#motor_pub.publish(msg)
		return 'confirm'

class Followline(Action):
	def __init__(self, speed = 0.5, timeout = DEFAULT_TIMEOUT):
		super(Followline, self).__init__(timeout = timeout)
		self.speed = speed

	def executeAction(self, userdata):
		global joyActive
		activate = Float32()
		if not joyActive:
			activate.data = self.speed
		else:
			activate.data = 0.0
		line_pub.publish(activate)
		return 'confirm'

class Spin(Action):
	def __init__(self, speed = 0.5, angle = 0, timeout = DEFAULT_TIMEOUT):
		super(Spin, self).__init__(timeout = timeout)
		self.speed = speed
		self.angle = angle

	def executeAction(self, userdata):
		#msg = MotorState()
		#msg.speed = self.speed
		#msg.heading_angle = self.angle
		#msg.mode = msg.DRIVE_MODE_SPIN
		#motor_pub.publish(msg)
		return 'confirm'


#class Turn(Action):
#	def __init__(self, speed = 0.1, timeout = DEFAULT_TIMEOUT):
#		super(Turn, self).__init__(timeout = timeout)
#		self.speed = speed
#
#	def executeAction(self, userdata):
#		rospy.loginfo('send turn command')
#		#pub.publish(str(self.speed))
#		return 'confirm'


# CHECK STEPS
class CheckCrossing(Check):
    def execute(self, userdata):
		#rospy.loginfo("checkCrossing @ " + str(robot));
		return False
		#return robot[0] == 5

class CheckWaitFor(Check):
    def execute(self, userdata):
    	#rospy.loginfo("checkWaitFor @ " + str(robot));
    	return False
    	#return random.randint(0,10)>5

class CheckLine(Check):
    def execute(self, userdata):
    	#rospy.loginfo("checkLine @ " + str(robot));
    	return False
    	#return random.randint(0,10)>5

class CheckJoy(Check):
    def execute(self, userdata):
    	return joyActive



class CheckForSpin(Check):
	def execute(self, userdata):
		if spin_done:
			return True
		else:
			return False



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
        smach.StateMachine.add('Followline', Followline(speed=0.4, timeout=1000000), 
                               transitions={'confirm':'CheckCrossing', 'timeout':'done'})
        smach.StateMachine.add('CheckCrossing', CheckCrossing(), 
                               transitions={True:'done', False:"Followline"})
        #smach.StateMachine.add('CheckWaitFor', CheckWaitFor(), 
        #                       transitions={True:'done', False:"Followline"})
    return sm

def createlineSM2():
    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('Followline', Followline(speed=0.2, timeout=10), 
                               transitions={'confirm':'CheckCrossing', 'timeout':'done'})
        smach.StateMachine.add('CheckCrossing', CheckCrossing(), 
                               transitions={True:'done', False:"Followline"})
        #smach.StateMachine.add('CheckWaitFor', CheckWaitFor(), 
        #                       transitions={True:'done', False:"Followline"})
    return sm

def createlineSM3():
    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('Followline', Followline(speed=0.6, timeout=10), 
                               transitions={'confirm':'CheckCrossing', 'timeout':'done'})
        smach.StateMachine.add('CheckCrossing', CheckCrossing(), 
                               transitions={True:'done', False:"Followline"})
        #smach.StateMachine.add('CheckWaitFor', CheckWaitFor(), 
        #                       transitions={True:'done', False:"Followline"})
    return sm

def spinTestSM():
    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('Spin', Spin(speed=0.75, angle=90, timeout=1000000), 
                               transitions={'confirm':'CheckForSpin', 'timeout':'done'})
        smach.StateMachine.add('CheckForSpin', CheckForSpin(), 
                               transitions={True:'Drive', False:"CheckForSpin"})
        smach.StateMachine.add('Drive', Drive(speed=0.0, timeout=1000000), 
                               transitions={'confirm':'done', 'timeout':'done'})
    return sm

#def createlineSM2():
#    sm = smach.StateMachine(outcomes=['done'])
#    with sm:
#        # Add states to the container
#        smach.StateMachine.add('Turn', Turn(timeout = 50), 
#                               transitions={'confirm':'CheckCrossing', 'timeout':'done'})
#        smach.StateMachine.add('CheckCrossing', CheckCrossing(), 
#                               transitions={True:'done', False:"CheckWaitFor"})
#        smach.StateMachine.add('CheckWaitFor', CheckWaitFor(), 
#                               transitions={True:'done', False:"Turn"})
#    return sm



def main():
    rospy.init_node('smach_example_state_machine')
    global rate
    rate = rospy.Rate(10)
    global joyTime
    joyTime = rospy.Time.now()

    #rospy.Subscriber("odom_vel", Float32MultiArray, spin_callback)
    rospy.Subscriber("cmd_joy_vel", MotorState, joy_callback)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:
    	smach.StateMachine.add('checkJoy', CheckJoy(), 
                               transitions={True:'SUB', False:"checkJoy"})
        
        smach.StateMachine.add('SUB', createlineSM(),
        						transitions={'done':'done'})
        smach.StateMachine.add('SUB2', createlineSM2(),
        						transitions={'done':'done'})
        smach.StateMachine.add('SUB3', createlineSM3(),
        						transitions={'done':'done'})
        #smach.StateMachine.add('RateChange', RateChange(3),
       # 						transitions={'done':'SUB2'})
       # smach.StateMachine.add('SUB2', createlineSM2(),
       # 						transitions={'done':'done'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
