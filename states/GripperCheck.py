#!/usr/bin/env python

import rospy
import sys
import copy
import time
import std_msgs.msg
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

'''
Created on 04.05.2017

@author: Alessio Caporali
'''

class GripperStateEffort(EventState):
	'''
	Open/Close the gripper based on the input value width.
	Check if the gripper succeded in grasping the cube by looking at the effort value.

	-- width	float		Wideness of the gripper motion.
	-- threshold	float		Value of torque above which we are sure(5 = safe).
	-- time		float	 	Supposed time needed to complete the task.  

	<= done				Cube grasped successfully.
	<= failed 		    	Failed to grasp the cube.
	'''

	def __init__(self, width, threshold, time):
		'''
		Constructor
		'''
		super(GripperStateEffort, self).__init__(outcomes=['done', 'failed'])
		rospy.Subscriber('/joint_states', JointState, self.CallbackJoints)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		self._target_time = time
		self._width = width
		self._threshold = threshold


	def CallbackJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)
		self.gripper_effort = [self.joint_states.effort[15], self.joint_states.effort[16]]
	
	def execute(self, userdata):
		timer = rospy.get_time() - self._start_time
		print "Timer execution: ",timer

		if timer < self._target_time:
			if (self.gripper_effort[0] > self._threshold) and (self.gripper_effort[1] > self._threshold):
				return 'done'
		else:
			return 'failed'

	def on_enter(self, userdata):

		self._start_time = rospy.get_time()

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
		Logger.loginfo('Exiting Gripper Effort State')

