#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Twist, PoseStamped
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
import logging
import copy
import numpy
import time

class switch(object):
	value = None
        def __new__(class_, value):
        	class_.value = value
        	return True

def case(*args):
	return any((arg == switch.value for arg in args))

class pose_listener:
	
	
	def __init__(self):
		rospy.init_node('pose_listener', anonymous = True)
		rospy.Subscriber('pose_command', Twist, self.processCmd)
		rospy.Subscriber('move_base/status', GoalStatusArray, self.readStatus)
		self.goalPub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
		self.status = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.doPub = rospy.Publisher('/done_pose', Int8, queue_size=10)
		
		self.done = 1
		
		self.value = [0 for i in range(2)]
		self.desiredposition = PoseStamped()
		self.status.wait_for_server(rospy.Duration(5))
		
		
	def processCmd(self, cmd_vel):
		self.cmd = cmd_vel
		value = [0 for i in range(3)]
		value[0] = self.cmd.linear.x  # x coordinate of goal
		value[1] = self.cmd.linear.y  # y coordinate of goal
		value[2] = self.cmd.angular.z # orientation				

		self.desiredposition.header.stamp = rospy.Time.now()
		self.desiredposition.header.frame_id = 'map'
		self.desiredposition.pose.position.x = value[0]	
		self.desiredposition.pose.position.y = value[1]
		self.desiredposition.pose.orientation.z = value[2]
		self.desiredposition.pose.orientation.w = 1.0
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
        	goal.target_pose.pose = Pose(Point(value[0], value[1], 0.000), Quaternion(0.0, 0.0, value[2], 1.0))

		#self.goalPub.publish(self.desiredposition)
		self.status.send_goal(goal)	

	def readStatus(self, move_base):
		self.move_base = copy.deepcopy(move_base)
		
		#self.status.wait_for_result(rospy.Duration(60))
		
		#time.sleep(2)	
		print(self.status.get_state())


		if self.status.get_state() != 3:
			self.done = 0
		else:
			self.done = 1	

		print(self.done)

		self.doPub.publish(self.done)

def main(args):
	logging.disable(logging.CRITICAL)
	pose_listener()
	# rospy.init_node('pose_listener', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	# rospy.Subscriber('move_base/status', GoalStatusArray, pose_listener.readStatus)
	main(sys.argv)
