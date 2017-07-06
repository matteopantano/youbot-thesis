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

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint




class GripperStateOpen(EventState):
	'''
	Open the gripper.
	-- close_fraction 	float 	OPEN, CLOSE or any value in [0,1].
	-- duration 		float 	Time (sec) for executing the motion.
	<= done 					Trajectory has successfully finished its execution.
	<= failed 					Failed to execute trajectory.
	'''
	OPEN = 0
	CLOSE = 1

	def __init__(self, duration = 1.0):
		'''
		Constructor
		'''
		super(GripperStateOpen, self).__init__(outcomes=['done', 'failed'])
		rospy.Subscriber('/joint_states', JointState, self.CallbackJoints)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		self.duration = duration


	def CallbackJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)

	
	def execute(self, userdata):

		current_value_l = abs(0.0115 - self.joint_states.position[15])	
		current_value_r = abs(0.0115 - self.joint_states.position[16])		
		Logger.loginfo(current_value_l)
		Logger.loginfo(current_value_r)

		if (current_value_l <= 0.001) and (current_value_r <= 0.001):
			return 'done'
		

		
	def on_enter(self, userdata):

		desiredPositions = JointPositions()

        	jointCommands = []

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_l"
        	joint.unit = "m"
        	joint.value = 0.0115
        	jointCommands.append(joint)

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_r"
        	joint.unit = "m"
        	joint.value = 0.0115
        	jointCommands.append(joint)

        	desiredPositions.positions = jointCommands

        	self.gripPub.publish(desiredPositions)


	def on_exit(self, userdata):
		Logger.loginfo('Exiting Gripper State Open')

