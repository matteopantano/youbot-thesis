#!/usr/bin/env python

import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
import sys
import copy
import numpy as np
import math
import time
import logging
import math

class arm_kinematics:
		
	def __init__(self):
		rospy.init_node('arm_kinematics', anonymous = False)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		rospy.Subscriber('/box_pose', Pose, self.kinematics)
		
		self.jointHome = [3.0, 1.25, -1, 2.8, 3.07]
		self.openGripper = 0.0114
		self.closeGripper = 0.0012
		self.uri = 0
		self.value = [0 for i in range(6)]

	def movegripper(self, value):
		jh = JointPositions()
		jg = JointValue()
            	jg.joint_uri = "gripper_finger_joint_r"
            	jg.unit = "m"
            	jg.value = value
            	jh.positions.append(jg)
		self.gripPub.publish(jh)
		rospy.sleep(0.2)
            	jg.joint_uri = "gripper_finger_joint_l"
            	jg.value = value
            	jh.positions.append(jg)
		self.gripPub.publish(jh)

	def movearm(self, value):
		jp = JointPositions()
		for i in range(5):
			jv = JointValue()
			jv.joint_uri = "arm_joint_" + str(i + 1)
			jv.unit = "rad"
			jv.value = value[i]
			jp.positions.append(jv)

		rospy.sleep(0.5)
		self.armPub.publish(jp)

	def kinematics(self, pose):
		
		pose_target = Pose()

		pose_target.orientation.x = pose.orientation.x
  		pose_target.orientation.y = pose.orientation.y
  		pose_target.orientation.z = pose.orientation.z
  		pose_target.orientation.w = pose.orientation.w
		
		# adjust above cube
  		pose_target.position.x = pose.position.x
  		pose_target.position.y = pose.position.y
  		pose_target.position.z = pose.position.z

		# height of arm_joint_2 respect to arm_joint_4
		height = 57

		# distance of the object related to arm_joint_2
		distance = pose.position.x - 200

		h = math.sqrt(distance**2 + height**2)

		# small rectangualr triangle, with an angle of 90 degrees
		c = math.asin(height/h)
		d = math.pi/2 - c

		# computed angle of arm_joint_3, NOT THE ONE TO SEND
		beta = math.acos((155**2+135**2 - h**2)/(2*155*135))

		# computed angle of x (see drawing)
		x = math.asin((155*math.sin(beta))/h)
		X = x + d

		# computed angle of alfa (see drawing)
		alfa = math.pi - beta - x
		ALFA = alfa + c

		# TRANSFORM
		joint_2 = 2.70 - ALFA
		joint_3 = -(beta - 0.53)
		joint_4 = 1.7889 * 2 - (X - math.pi - 1.7889 * 2)

		print(joint_2, joint_3, joint_4)

  		
		

		# above grasp
		# self.movearm(self.value)
		


def main(args):
	arm_kinematics()
	try:		
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
