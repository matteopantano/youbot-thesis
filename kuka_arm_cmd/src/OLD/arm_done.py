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

def truncate(x, d):
	return int (x*(10.0**d))/(10.0**d)

class arm_done:
	
	
	def __init__(self):
		rospy.init_node('arm_done', anonymous = True)	
		rospy.Subscriber('arm_controller/position_command', JointPositions, self.processPositions)
		#rospy.Subscriber('gripper_controller/position_command', JointPositions, self.processGripper)
		rospy.Subscriber('joint_states', JointState, self.processJoints)
		self.doPub = rospy.Publisher('/done_arm', Int8, queue_size=10)

		self.value = [0 for i in range(6)]
		self.joint_states = JointState()	
		self.done = 0

	def processPositions(self, joint_positions):
		self.joint_positions = joint_positions
		#if self.first:
		#	self.value[0] = 1.0
		#	self.value[1] = 1.0
		#	self.value[2] = -2.0
		#	self.value[3] = 1.0
		#	self.value[4] = 4.0
		#	self.first = 0
		#else:
		self.value[0] = 4.0
		self.value[1] = 1.0
		self.value[2] = -2.0
		self.value[3] = 1.0
		self.value[4] = 4.0


	def processJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)

		if (abs(self.value[0] - self.joint_states.position[10])) <= 0.015:
			self.done = self.done + 1
		else:
			self.done = self.done
		if (abs(self.value[1] - self.joint_states.position[11])) <= 0.015:
			self.done = self.done + 1
		else:
			self.done = self.done
		if (abs(self.value[2] - self.joint_states.position[12])) <= 0.015:
			self.done = self.done + 1
		else:
			self.done = self.done
		if (abs(self.value[3] - self.joint_states.position[13])) <= 0.015:
			self.done = self.done + 1
		else:
			self.done = self.done
		if (abs(self.value[4] - self.joint_states.position[14])) <= 0.015:
			self.done = self.done + 1
		else:
			self.done = self.done
		#if (abs(self.value[5] - self.joint_states.position[15]) <= 0.001) and (abs(self.value[5] - self.joint_states.position[16]) <= 0.001):
		#	self.done = self.done + 1
		#else:
		#	self.done = self.done		

			
                #time.sleep(10)	
		#print(self.done)			
		if self.done == 5:	
			print("Sono qui -- DONE")
			self.doPub.publish(1)
		else:
			print("Sono qui ++ NO")
			self.doPub.publish(0)
		#time.sleep(2)
		#print("================================AZZERAMENTO==========================================")
		self.done = 0
		

def main(args):
	arm_done()
	# rospy.init_node('arm_done', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
		
