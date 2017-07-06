#!/usr/bin/env python

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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

'''
Created on 03.04.2017

@author: Matteo Pantano
'''

class GripperStateWidth(EventState):
	'''
	Open/Close the gripper based on the input value width.

	-- width	 	float	    Wideness of the gripper motion.

	<= done 		            Trajectory has successfully finished its execution.
	<= failed 			    Failed to execute trajectory.
	'''

	def __init__(self, width):
		'''
		Constructor
		'''
		super(GripperStateWidth, self).__init__(outcomes=['done', 'failed'])
		rospy.Subscriber('/joint_states', JointState, self.CallbackJoints)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		self.duration = 1.0
		self._width = width


	def CallbackJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)

	
	def execute(self, userdata):

		current_value_l = abs(self.value - self.joint_states.position[15])	
		current_value_r = abs(self.value - self.joint_states.position[16])

		if (current_value_l <= 0.002) and (current_value_r <= 0.002):
			return 'done'
		

	def on_enter(self, userdata):
		
		self.value = self._width
		desiredPositions = JointPositions()

        	jointCommands = []

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_l"
        	joint.unit = "m"
        	joint.value = self.value
        	jointCommands.append(joint)

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_r"
        	joint.unit = "m"
        	joint.value = self.value
        	jointCommands.append(joint)

        	desiredPositions.positions = jointCommands

        	self.gripPub.publish(desiredPositions)


	def on_exit(self, userdata):
		Logger.loginfo('Exiting Gripper State Width')

