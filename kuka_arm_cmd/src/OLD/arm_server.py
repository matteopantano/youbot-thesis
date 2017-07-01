#!/usr/bin/env python

import roslib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
 
class Arm:
	def __init__(self, arm_name):
        	#arm_name should be l_arm or r_arm
            	self.name = arm_name
           	self.jta = actionlib.SimpleActionClient("arm_controller",JointTrajectoryAction)
            	rospy.loginfo('Waiting for joint trajectory action')
            	self.jta.wait_for_server()
            	rospy.loginfo('Found joint trajectory action!')
 
        def move(self, angles):
            	goal = JointTrajectoryGoal()
            	goal.trajectory.joint_names = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5']
            	point = JointTrajectoryPoint()
            	point.positions = angles
            	point.time_from_start = rospy.Duration(3)
            	goal.trajectory.points.append(point)
            	self.jta.send_goal_and_wait(goal)
 
def main():
	arm = Arm('arm')
        arm.move([1.0,1.0,-2.0,1.0,3.0])
 
 
if __name__ == '__main__':
	rospy.init_node('joint_position_tester')
      	main()		
