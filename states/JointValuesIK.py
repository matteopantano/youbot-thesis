#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# Additional imports can be added inside the following tags
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
from geometry_msgs.msg import Pose, Pose2D
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
# [/MANUAL_IMPORT]

'''
Created on 02.04.2017

@author: Matteo Pantano
'''

class JointValueIK(EventState):
	'''
	Get IK solution for the box pose in terms of joint values.

	-- above         float      Height at which the IK solution should stop.
	-- grasp_angle   float      Inclination of the end effector wrt the ground, radiants.

	># pose          Pose       Target for IK.
	#> output_ik	 object     The message containing joint values.

	<= found 		    IK solution found.
	<= unavailable 		    IK soolution not fould.
	'''


	def __init__(self, above, grasp_angle):
		'''
		Constructor
		'''
		super(JointValueIK, self).__init__(outcomes=['found', 'unavailable'], output_keys=['output_ik'], input_keys = ['pose'])

		self.value = [0 for i in range(6)]
		self._height = above
		self._inclination = grasp_angle
	
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if not self.value:
			return 'unavailable'
		else:
			Logger.logwarn(self.value)
			userdata.output_ik = self.value
			return'found'
		

	def on_enter(self, userdata):
		height =  self._height
		inclination = self._inclination
				

		pose_orientation_x = userdata.pose.orientation.x
  		pose_orientation_y = userdata.pose.orientation.y
  		pose_orientation_z = userdata.pose.orientation.z
  		pose_orientation_w = userdata.pose.orientation.w

		(roll,pitch,yaw) = euler_from_quaternion([pose_orientation_x, pose_orientation_y, pose_orientation_z, pose_orientation_w])

		rotation = yaw + 0.01

		self.value = self.inverse(userdata.pose.position, height, inclination, rotation)     	


	def inverse(self, p, h, inclination_link, rotation_cube):
		
		#print("inside inverse kinematics function")
				
		pose_x = p.x * 1000
		pose_y = p.y * 1000 
		pose_z = p.z * 1000
		gamma = inclination_link
		teta_5 = rotation_cube
	
		if rotation_cube < -1.57:
			teta_5 = rotation_cube + 1.57
		if rotation_cube > 1.57:
			teta_5 = rotation_cube - 1.57
		else:
		 	teta_5 = rotation_cube

		link_12_xy = 33 # vector btw arm_link_1 and arm_link_2
		
		#teta_1 calculation
		zeta = math.atan2(pose_y,(pose_x - 167))
		#print("zeta:",zeta)

		arm_2_x = link_12_xy * math.cos(-zeta)
		arm_2_y = link_12_xy * math.sin(-zeta)
	

		# solving inverse kinematics 3R planar manipulator  -- (Kumar PDF
		a = math.sqrt((pose_x - 167 - arm_2_x)**2 + (pose_y + arm_2_y)**2)
		l3 = 176
		l2 = 135
		l1 = 155
		y = - 205 + abs(pose_z) - (h * 100)

		#print(a,y)
		
		#print("End-Effector final position: ", pose_x, pose_y, y)		
		

		fi = - inclination_link
		X = a - l3*math.cos(fi)
		Y = y - l3*math.sin(fi)
		p = math.sqrt(X**2 + Y**2)

		# calculate angle teta1 (joint_2)
		ro = math.atan2( (-Y/p), (-X/p) )
		t1_partial = math.acos(-(X**2 + Y**2 + l1**2 - l2**2)/(2*l1*math.sqrt(X**2 + Y**2)))
		t1 = ro - t1_partial
		
		if  ((t1 > 2.70) or (t1 < 0)):
			t1 = ro + t1_partial
		
		# calculate angle teta2	(joint_3)
		y2 = (Y - l1*math.sin(t1))/l2
		x2 = (X - l1*math.cos(t1) )/l2
		t2 = math.atan2(y2,x2) - t1

		#calculate angle teta3 (joint_4)
		t3 = fi - t1 - t2
		t2 = t2
		t3 = t3

		t1deg = math.degrees(t1)
		t2deg = math.degrees(t2)
		t3deg = math.degrees(t3)
		#print(" Joint 2-3-4 in degrees:" ,t1deg, t2deg, t3deg)

		teta_1 = zeta
		teta_tool = gamma

		#-----------------------------
		# MAPPING for Vrep
		joint_1 = 2.94961 - teta_1
		joint_2 = 2.70526 - t1
		joint_3 = 0.523599 - (3.14 + t2)
		joint_4 = 4.930555 - (3.14 + t3)
		joint_5 = 2.92 + teta_5 - zeta 

		if joint_5 < 0:
			joint_5 = joint_5 + 3.14
		if joint_5 > 4.71:
			joint_5 = joint_5 - 3.14
		
		joint_q = (joint_1, joint_2, joint_3, joint_4, joint_5)

		return joint_q

	def on_exit(self, userdata):
		Logger.loginfo('Exiting JointValuesIK State')
