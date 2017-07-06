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
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import euler_from_quaternion

from lib_inverse_kinematics	import Inverse_Kinematics 
# [/MANUAL_IMPORT]

'''
Created on 03.04.2017

@author: Alessio Caporali
'''

class IKSolverTrajectory(EventState):
	'''
	Get IK solution for the box pose and execute trajectory.

	--hertz		float	    Loop frequency for publishing each point of the trajectory.		
	--samples	float	    Number of points in the trajectory.
	--offset	float	    How much go down to grasp the cube.
	--inclination_ee float	    Inclination of the End_Effector with respect to the ground.

	># pose          Pose       Target for IK.

	<= found 		    IK solution found.
	<= unavailable 		    IK soolution not fould.
	'''


	def __init__(self, hertz, samples, offset, inclination_ee):
		'''
		Constructor
		'''
		super(IKSolverTrajectory, self).__init__(outcomes=['found', 'unavailable'], input_keys = ['pose'])
		rospy.Subscriber('/joint_states', JointState, self.CallbackJoints)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		rospy.Subscriber('/ee_pose', Pose, self.end_effector_pose)

		self.done = 0
		self.ok = 0

		self.value = [0 for i in range(6)]
		self.hertz = hertz		
		self.samples = samples
		self.offset = offset
		self.inclination_ee = - inclination_ee



	def CallbackJoints(self, joint_states):		
		self.joint_states = copy.deepcopy(joint_states)

	def end_effector_pose(self, pose):
		ee_p =[pose.position.x, pose.position.y, pose.position.z]
		self.end_effector = ee_p

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

	def trajectory_calc(self, end_effector, target, number, offset_grasp):
		number_real = number + number/5
		self.q = [0 for i in range(number_real)]

		v = [end_effector[0] - target[0], end_effector[1] - target[1], end_effector[2] - abs(target[2])]
		dim =  np.linalg.norm(v)
		print ("Magnitude of Vector: ", dim) 
	
		i = 0
		k = 0
		t = 0
		inc = 1.0/number
		for i in range(number):
			i = i + 1
			t = 0 + i*inc 
			p = np.array(end_effector) - (t*np.array(v))
			self.q[i-1] = Inverse_Kinematics(p, self.inclination_ee, self.rotation)

		i = 0
		inc2 = 1.0/(number/5.0)
		for i in range(number/5):
			i = i + 1
			k = 0 + i*inc2
			p2 = [p[0], p[1], (p[2] - k*(offset_grasp/1000.0))]
			self.q[i-1+number] = Inverse_Kinematics(p2, self.inclination_ee, self.rotation)


	
	def trajectory_pub(self, joint_q, frequency, samples):
		samples_tot = samples + samples/5
		r = rospy.Rate(frequency)
		i = 0
		for i in range(samples_tot):
			i = i + 1 
			self.publish_arm_joint_positions(joint_q[i-1])
			r.sleep()

		return 1



	
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
		Logger.logwarn('POSE OBJECT: ')
		Logger.logwarn(userdata.pose.position)
		
		self.target = [userdata.pose.position.x, userdata.pose.position.y, userdata.pose.position.z]
		(roll,pitch,yaw) = euler_from_quaternion([userdata.pose.orientation.x, userdata.pose.orientation.y, userdata.pose.orientation.z, userdata.pose.orientation.w])
		
		self.rotation = yaw
		

		self.trajectory_calc(self.end_effector, self.target, self.samples, self.offset)
		n = (self.samples + self.samples/5) -1
		self.value = self.q[n]

		self.ok = self.trajectory_pub(self.q, self.hertz, self.samples)



	



	def on_exit(self, userdata):
		Logger.loginfo('Exiting JointValuesIK State')
