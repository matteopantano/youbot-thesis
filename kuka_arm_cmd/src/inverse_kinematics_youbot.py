#!/usr/bin/env python

import rospy
import sys
import copy
import numpy as np
import math
import time
import logging
import std_msgs.msg
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion



class arm_kinematics:
	
	

	def __init__(self):
  		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('arm_kinematics', anonymous = True)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		rospy.Subscriber('/box_pose', Pose, self.kinematics)

		
		self.home = [3.0, 1.25, -1.0, 2.8, 3.07]
		self.gripperWidthAtGrasp = 0.0012
		self.value = [0 for i in range(6)]

	def publish_arm_joint_positions(self, joint_positions):

        	desiredPositions = JointPositions()

        	jointCommands = []

        	for i in range(5):
            		joint = JointValue()
            		joint.joint_uri = "arm_joint_" + str(i+1)
            		joint.unit = "rad"
            		joint.value = joint_positions[i]

           		jointCommands.append(joint)
            
        	desiredPositions.positions = jointCommands

       		self.armPub.publish(desiredPositions)


                  
	def publish_gripper_width(self, width):
                  
       		desiredPositions = JointPositions()

        	jointCommands = []

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_l"
        	joint.unit = "m"
        	joint.value = width
        	jointCommands.append(joint)

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_r"
        	joint.unit = "m"
        	joint.value = width
        	jointCommands.append(joint)

        	desiredPositions.positions = jointCommands

        	self.gripPub.publish(desiredPositions)

	def inverse(self, p, h, inclination_link, rotation_cube):
		
		print("sono dentro")
				
		pose_x = p.x * 1000
		pose_y = p.y * 1000
		pose_z = p.z * 1000
		h = h*1000 # height above the cube 
		gamma = inclination_link
		teta_5 = rotation_cube
		b = 176
		c = 135
		d = 155
		e = 230 - (pose_z + h) 
		link_12_xy = 33 # vector btw arm_link_1 and arm_link_2
		
		#teta_1 calculation
		zeta = math.atan2(pose_y,(pose_x - 167))
		print(zeta)

		arm_2_x = link_12_xy * math.cos(-zeta)
		arm_2_y = link_12_xy * math.sin(-zeta)
	
		
		# triangle AED
		a = math.sqrt((pose_x - 161 - arm_2_x)**2 + (pose_y + arm_2_y)**2) 
		f = math.sqrt(a**2 + b**2 -2*(a*b*math.cos(gamma)))

		alfa = math.asin((b*math.sin(gamma))/f)
		beta = 3.14 - alfa - gamma #beta = math.asin((a*math.sin(gamma))/f)

		# triangle ADB
		epsilon = 1.57 - alfa
		g = math.sqrt(e**2 + f**2 -2*(e*f*math.cos(epsilon)))

		omega = math.asin((f*math.sin(epsilon))/g)
		fi = math.asin((e*math.sin(epsilon))/g)

		#triangle BCD
		sigma = math.acos((c**2 + g**2 - d**2)/(2*g*c))
		psi = math.asin((c*math.sin(sigma))/d)
		delta = 3.14 - sigma - psi

		teta_1 = zeta
		teta_2 = omega + psi
		teta_3 = delta
		teta_4 = sigma + fi + beta
		teta_tool = gamma

		print(teta_2, teta_3, teta_4)
		# MAPPING for Vrep

		joint_1 = 2.94961 - teta_1
		joint_2 = 2.70526 - (teta_2 - 1.57)
		joint_3 = 0.523599 - teta_3 
		joint_4 = 4.930555 - teta_4
		joint_5 = 2.94961 - teta_5
		
		joint_q = (joint_1, joint_2, joint_3, joint_4, joint_5)
		
		return joint_q


	def kinematics(self, pose):


		#print(pose)
		# orientation
		pose_orientation_x = pose.orientation.x
  		pose_orientation_y = pose.orientation.y
  		pose_orientation_z = pose.orientation.z
  		pose_orientation_w = pose.orientation.w

		(roll,pitch,yaw) = euler_from_quaternion([pose_orientation_x, pose_orientation_y, pose_orientation_z, pose_orientation_w])

		print 'YAW ---', yaw
		
 
		height =  0.02
		inclination = 1.57
		rotation = 0		

		self.value = self.inverse(pose.position, height, inclination, rotation)
		print(self.value)
		rospy.sleep(0.5)
		#self.publish_arm_joint_positions(self.value)    
		
		

def main(args):

	arm_kinematics()
	logging.disable(logging.CRITICAL)

	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
