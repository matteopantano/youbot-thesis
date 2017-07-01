#!/usr/bin/env python  

import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('tf_box_pose')
    pub = rospy.Publisher('/box_pose', Pose, queue_size=10)
    listener = tf.TransformListener()
    pose = Pose()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_footprint', '/box', rospy.Time(0))
	    #print(trans,rot)
	    pose.position.x = trans[0]
	    pose.position.y = trans[1]
	    pose.position.z = trans[2]

	    pose.orientation.x = rot[0]
	    pose.orientation.y = rot[1]
	    pose.orientation.z = rot[2]
	    pose.orientation.w = rot[3]

	    print(pose)
	    pub.publish(pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

