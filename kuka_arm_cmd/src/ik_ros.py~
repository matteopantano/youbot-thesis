#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
import sys
import copy
import numpy as np
import time
import ikpy 
from ikpy import plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

def limit(value, max_limits, low_limits):

	if value[0] < max_limits[0] or value[0] > low_limits[0]:
		print("--1--")
		return 1
	if value[1] > max_limits[1] or value[1] < low_limits[1]:
		print("--2--")
		return 1	
  	if value[2] < max_limits[2] or value[2] > low_limits[2]:
		print("--3--")
		return 1
	if value[3] > max_limits[3] or value[3] < low_limits[3]:
		print("--4--")
		return 1
	if value[4] < max_limits[4] or value[4] > low_limits[4]:
		print("--5--")
		return 1
	
	return 0	

def mapping(value):
	
	value[0] = abs(value[0])    # joint_0
	
	value[1] = value[1]
	
  	value[2] = value[2]
	
	value[3] = value[3]

	value[4] = 2.92345 - value[4]

	return value	

class ik:
	
	def __init__(self):
		self.done = 0
		rospy.init_node('ik', anonymous = True)
		#self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		#self.doPub = rospy.Publisher('/ik_result', Int8, queue_size=10)
			
		self.desiredposition = JointPositions()	

		self.jointBoundsUp = [-2.9496, 1.5708, -2.6354, 1.7890, -2.9234]
		self.jointBoundsLow = [2.9496, -1.1345, 2.5482, -1.7890, 2.9234]
		self.process_ik()

	def process_ik(self):

		my_chain = ikpy.chain.Chain.from_urdf_file("/home/matteo/rover_ws/urdf/youbot.urdf", base_elements=["base_link", "arm_joint_0", "arm_link_0", "arm_joint_1", "arm_link_1", "arm_joint_2", "arm_link_2", "arm_joint_3", "arm_link_3", "arm_joint_4", "arm_link_4", "arm_joint_5", "arm_link_5"])
		ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

		target =[0.4, 0.0, 0.1]
		frame_target = np.eye(4)
		frame_target[:3, 3] = target

		joints = [0] * len(my_chain.links)
		ik = my_chain.inverse_kinematics(frame_target, initial_position=joints)

		print(ik)		

		self.ik = [ik[2], ik[3], ik[4], ik[5], ik[6]]	

		print ("SONO QUI")	

		#for index in range(5):
			#unit[index] = "rad"
			#unit[5]="m"	

		invalid = limit(self.ik, self.jointBoundsUp, self.jointBoundsLow)

		if invalid == 0:
       			#for index in range(0, 5):
				#pos = JointValue()
				#pos.joint_uri = "arm_joint_" + str(index + 1)
				#pos.unit = unit[index]
				#pos.value = value[index]
				#self.desiredposition.positions.append(pos)
	
				#print("------------------------------------------------")
		   		#print(self.desiredposition.positions)
				
				print("VALID IK SOLUTION")
				
				self.ik = mapping(self.ik)
			
				print(self.ik)

				#self.armPub.publish(self.desiredposition)	
		else:
				print("INVALID IK SOLUTION")	
		
		my_chain.plot(ik, ax, target=target)
		matplotlib.pyplot.show()		


	def limit(self, value, max_limits, low_limits):

		if value[0] < max_limits[0] or value[0] > low_limits[0]:
			return 1
		if value[1] > max_limits[1] or value[1] < low_limits[1]:
			return 1	
	  	if value[2] < max_limits[2] or value[2] > low_limits[2]:
			return 1
		if value[3] > max_limits[3] or value[3] < low_limits[3]:
			return 1
		if value[4] < max_limits[4] or value[4] > low_limits[4]:
			return 1
	
		return 0
		

def main(args):
	ik()
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
		
