#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import Joy		
from geometry_msgs.msg import Twist	

distance=Twist()

rospy.init_node('teleop', anonymous=True)

distance.linear.x = 0
distance.linear.y= 0	
distance.angular.z = 0

def callback(data):
	global distance				 
	rospy.loginfo(data)				   	
	distance.linear.x=data.axes[1]
        distance.linear.y=data.axes[0]
	distance.angular.z=data.axes[2]

rospy.Subscriber("joy", Joy, callback)		
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)	

rate = rospy.Rate(10)

while not rospy.is_shutdown():
	pub.publish(distance)		
	rate.sleep()
