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
		self.done = 0
		rospy.init_node('arm_listener', anonymous = True)
		rospy.Subscriber('arm_command', Twist, self.processCmd)
		rospy.Subscriber('joint_states', JointState, self.processJoints)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		self.doPub = rospy.Publisher('/done_arm', Int8, queue_size=10)
		
		self.pos = JointValue()
		self.uri = 0
		self.value = [0 for i in range(6)]
		self.joint_states = JointState()		
		self.desiredposition = JointPositions()	
		self.once = 1

		self.jointBoundsUp = [5.8, 2.6, -0.015708, 3.40, 5.60, 0.0115]
		self.jointBoundsLow = [0.011, 0.020, -5.02654, 0.03, 0.12, 0.0]
		 #time.sleep(2)

	def processCmd(self, cmd_vel):

		
		self.cmd = cmd_vel
		value = [0 for i in range(6)]
		unit = [0 for i in range(6)]
		value[0] = self.cmd.linear.x  # joint_uri_1
		value[1] = self.cmd.linear.y  # joint_uri_2
		value[2] = self.cmd.linear.z  # joint_uri_3 
		value[3] = self.cmd.angular.x # joint_uri_4
		value[4] = self.cmd.angular.y # joint_uri_5
		value[5] = self.cmd.angular.z # joint_uri_6	
		
		self.value = value

		print(self.value)

		for index in range(5):
			unit[index] = "rad"
		unit[5]="m"				

		for index in range(0, 6):
			if value[index] > self.jointBoundsUp[index]:
				value[index] = self.jointBoundsUp[index]
			elif value[index] < self.jointBoundsLow[index]:
				value[index] = self.jointBoundsLow[index]

       		for index in range(0, 5):
			pos = JointValue()
			pos.joint_uri = "arm_joint_" + str(index + 1)
			pos.unit = unit[index]
			pos.value = value[index]
			self.desiredposition.positions.append(pos)
	
		# print("------------------------------------------------")
   		# print(self.desiredposition.positions)

		self.armPub.publish(self.desiredposition)
		time.sleep(10)
		pos.joint_uri = "gripper_finger_joint_l" 
		pos.unit = unit[5]
		pos.value = value[5]
		self.desiredposition.positions.append(pos)
		self.gripPub.publish(self.desiredposition)
		time.sleep(0.2)
		pos.joint_uri = "gripper_finger_joint_r" 
		pos.unit = unit[5]
		pos.value = value[5]
		self.desiredposition.positions.append(pos)
		self.gripPub.publish(self.desiredposition)
		self.desiredposition = JointPositions()	

				
		

	def processJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)


		print("sono qui")


		#for index in range(0, 5):
		#time.sleep(0.06)
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
		if (abs(self.value[5] - self.joint_states.position[15]) <= 0.001) and (abs(self.value[5] - self.joint_states.position[16]) <= 0.001):
			self.done = self.done + 1
		else:
			self.done = self.done		

			
                #time.sleep(10)	
		#print(self.done)			
		if self.done == 6:	
			if self.once:
				self.doPub.publish(1)
				self.once = 0
		else:
			self.doPub.publish(0)
			self.once = 1
		#time.sleep(2)
		#print("================================AZZERAMENTO==========================================")
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
		
