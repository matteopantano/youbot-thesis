#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
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
# [/MANUAL_IMPORT]

'''
Created on 02.04.2017

@author: Matteo Pantano
'''

class JointValuePub(EventState):
	'''
	Publish Joint Values.

	-- target_pose   float[]    Target for the arm.

	<= done 		    Trajectory has successfully finished its execution.
	<= failed 		    Failed to execute trajectory.
	'''


	def __init__(self, target_pose):
		'''
		Constructor
		'''
		super(JointValuePub, self).__init__(outcomes=['done', 'failed'])
		rospy.Subscriber('/joint_states', JointState, self.CallbackJoints)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)

		self.jointBoundsUp = [5.8, 2.6, -0.015708, 3.40, 5.60, 0.0115]
		self.jointBoundsLow = [0.011, 0.020, -5.02654, 0.03, 0.12, 0.0]

		self._joint_positions = target_pose

		self.done = 0

	def CallbackJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)

	
	def execute(self, userdata):

		self.done = 0

		if (abs(self._joint_positions[0] - self.joint_states.position[10])) <= 0.016:
			self.done = self.done + 1

		if (abs(self._joint_positions[1] - self.joint_states.position[11])) <= 0.016:
			self.done = self.done + 1

		if (abs(self._joint_positions[2] - self.joint_states.position[12])) <= 0.016:
			self.done = self.done + 1

		if (abs(self._joint_positions[3] - self.joint_states.position[13])) <= 0.016:
			self.done = self.done + 1

		if (abs(self._joint_positions[4] - self.joint_states.position[14])) <= 0.016:
			self.done = self.done + 1


		if self.done == 5 :
			return 'done'		

	def on_enter(self, userdata):
				
		desiredPositions = JointPositions()

        	jointCommands = []

        	for i in range(5):
            		joint = JointValue()
            		joint.joint_uri = "arm_joint_" + str(i+1)
            		joint.unit = "rad"
            		joint.value = self._joint_positions[i]

           		jointCommands.append(joint)
            
        	desiredPositions.positions = jointCommands
       		self.armPub.publish(desiredPositions)

	def on_exit(self, userdata):
		Logger.loginfo('Exiting JointValuesIK State')

