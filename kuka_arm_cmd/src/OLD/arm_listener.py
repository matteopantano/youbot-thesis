#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
import sys
import copy
import numpy
import time


class switch(object):
	value = None
        def __new__(class_, value):
        	class_.value = value
        	return True

def case(*args):
    return any((arg == switch.value for arg in args))

class arm_listener:
	
	

	def __init__(self):
		rospy.init_node('arm_listener')
		rospy.Subscriber('arm_command', Twist, self.processCmd)
		rospy.Subscriber('joint_states', JointState, self.processJoints)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		# self.velPub = rospy.Publisher('/arm_controller/VELOCITY', JointVelocities, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		self.doPub = rospy.Publisher('/done', Int8, queue_size=10)

		self.done = 0
		self.pos = JointValue()
		self.uri = 0
		self.value = 0.0
		self.joint_states = JointState()		
		self.desiredposition = JointPositions()	

		self.jointBoundsUp = [5.8, 2.6, -0.015708, 3.40, 5.60, 0.0115, 0.0115]
		self.jointBoundsLow = [0.011, 0.020, -5.02654, 0.03, 0.12, 0.0, 0.0]

	def processCmd(self, cmd_vel):
		self.cmd = cmd_vel
		x = self.cmd.linear.x  # joint_uri
		y = self.cmd.linear.y  # value
		z = self.cmd.angular.z # unit 
		
		value = y

		# assign the name of the joint called and check consistency of value
		while switch(x):
    			if case(1):
        			joint_uri = "arm_joint_1"
				if value > self.jointBoundsUp[0]:
					value = self.jointBoundsUp[0]
				elif value < self.jointBoundsLow[0]:
					value = self.jointBoundsLow[0]
        			break
    			if case(2):
        			joint_uri = "arm_joint_2"
				if value > self.jointBoundsUp[1]:
					value = self.jointBoundsUp[1]
				elif value < self.jointBoundsLow[1]:
					value = self.jointBoundsLow[1]
       				break
    			if case(3):
        			joint_uri = "arm_joint_3"
				if value > self.jointBoundsUp[2]:
					value = self.jointBoundsUp[2]
				elif value < self.jointBoundsLow[2]:
					value = self.jointBoundsLow[2]
				break
    			if case(4):
        			joint_uri = "arm_joint_4"
				if value > self.jointBoundsUp[3]:
					value = self.jointBoundsUp[3]
				elif value < self.jointBoundsLow[3]:
					value = self.jointBoundsLow[3]
        			break
    			if case(5):
        			joint_uri = "arm_joint_5"
				if value > self.jointBoundsUp[4]:
					value = self.jointBoundsUp[4]
				elif value < self.jointBoundsLow[4]:
					value = self.jointBoundsLow[4]
        			break
			if case(6):
        			joint_uri = "gripper_finger_joint_r"
				if value > self.jointBoundsUp[5]:
					value = self.jointBoundsUp[5]
				elif value < self.jointBoundsLow[5]:
					value = self.jointBoundsLow[5]
        			break

		# assign the unit of measurament
		while switch(z):
    			if case(1):
        			unit = "rad"
        			break
    			if case(2):
        			unit = "m"
       				break
		

                self.pos.joint_uri = joint_uri
                self.pos.unit = unit
                self.pos.value = value
		if x == 6:
			self.desiredposition.positions.append(self.pos)
			self.gripPub.publish(self.desiredposition) 
			time.sleep(0.2)
                	self.pos.joint_uri = "gripper_finger_joint_l"
			self.desiredposition.positions.append(self.pos) 
			self.gripPub.publish(self.desiredposition)
		else:
                	self.desiredposition.positions.append(self.pos)

		self.uri = x
		self.value = value
		self.armPub.publish(self.desiredposition) 

	

	def processJoints(self, joint_states):	
		self.joint_states = copy.deepcopy(joint_states)

		while switch(self.uri):
    			if case(1):
        			if abs(self.value - self.joint_states.position[10]) <= 0.015:
					self.done = 1
				else:
					self.done = 0
				break
    			if case(2):
        			if abs(self.value - self.joint_states.position[11]) <= 0.015:
					self.done = 1
				else:
					self.done = 0
				break
    			if case(3):
        			if abs(self.value - self.joint_states.position[12]) <= 0.015:
					self.done = 1
				else:
					self.done = 0
				break
    			if case(4):
        			if abs(self.value - self.joint_states.position[13]) <= 0.015:
					self.done = 1
				else:
					self.done = 0
				break
    			if case(5):
        			if abs(self.value - self.joint_states.position[14]) <= 0.015:
					self.done = 1
				else:
					self.done = 0
				break
			if case(6):
        			if (abs(self.value - self.joint_states.position[15]) <= 0.001) and (abs(self.value - self.joint_states.position[16]) <= 0.001):
					self.done = 1
				else:
					self.done = 0
				break
        				
		
		self.doPub.publish(self.done)
		self.done = 0

def main(args):
	arm_listener()
	# rospy.init_node('arm_listener', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
		
