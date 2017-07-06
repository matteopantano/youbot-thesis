#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient


import rospy
import sys
import copy
import numpy as np
import math
import time
import logging
import std_msgs.msg
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities

from lib_inverse_kinematics	import Inverse_Kinematics 


'''
@author: Alessio Caporali
'''

class IKSolver(EventState):
	'''
	Compute Inverse Kinematics and execute the result.


	-- target_pose          float[]       Target for IK.
	-- inclination_ee	float	      Desired Inclination of the End_Effector with respect to the ground.	
	-- rotation_ee		float	      Rotation of the End_Effector (Joint5).

	<= found 			      IK solution found.
	<= unavailable 		              IK soolution not fould.
	'''


	def __init__(self, target_pose, inclination_ee, rotation_ee):
		'''
		Constructor
		'''
		super(IKSolver, self).__init__(outcomes=['found', 'unavailable'])
		rospy.Subscriber('/joint_states', JointState, self.CallbackJoints)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)

		self.done = 0
		self.ok = 0

		self.value = [0 for i in range(6)]
		self.target = target_pose
		self.inclination = - inclination_ee
		self.rotation = rotation_ee


	def CallbackJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)


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



	
	def execute(self, userdata):
		'''
		Execute this state
		'''
		self.done = 0
		if self.ok == 1:
			if (abs(self.value[0] - self.joint_states.position[10])) <= 0.015:
				self.done = self.done + 1

			if (abs(self.value[1] - self.joint_states.position[11])) <= 0.015:
				self.done = self.done + 1

			if (abs(self.value[2] - self.joint_states.position[12])) <= 0.015:
				self.done = self.done + 1

			if (abs(self.value[3] - self.joint_states.position[13])) <= 0.015:
				self.done = self.done + 1

			if (abs(self.value[4] - self.joint_states.position[14])) <= 0.015:
				self.done = self.done + 1
		else:
			pass


		if self.done == 5 :
			return 'found'
		else:	
			pass		


	def on_enter(self, userdata):
		
		' On Enter '
		self.value = Inverse_Kinematics(self.target, self.inclination, self.rotation)		
		self.publish_arm_joint_positions(self.value)
		rospy.sleep(0.2)
		self.ok = 1


	def on_exit(self, userdata):
		Logger.loginfo('Exiting IK_Solver State')

