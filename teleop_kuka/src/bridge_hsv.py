#!/usr/bin/python

import roslib
import rospy
import cv2
import cv2.cv as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import imutils
import tf

blueLower = (110, 50, 50)
blueUpper = (130, 255, 255)

class cvBridgeDemo():
    def __init__(self):
        
        rospy.init_node("blob_detection")
        rospy.on_shutdown(self.cleanup)
        cv.NamedWindow("RGB Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("RGB Image", 25, 75)
	cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
	cv.MoveWindow("Depth Image", 25, 350)
        
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgbimage/image_raw", Image, self.image_callback)
	self.image_sub = rospy.Subscriber("/kinect/depthimage/image_raw", Image, self.depth_callback)
        
        rospy.loginfo("Waiting for image topics...")

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        frame = np.array(frame, dtype=np.uint8)
        
        display_image = self.process_image(frame)
                       
        cv2.imshow("RGB Image", display_image)
        
        self.keystroke = cv.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                rospy.signal_shutdown("User hit q key to quit.")
                
            
    def process_image(self, frame):
         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   	 mask = cv2.inRange(hsv, blueLower, blueUpper)
         mask = cv2.erode(mask, None, iterations=2)
   	 mask = cv2.dilate(mask, None, iterations=2)

    	 cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    	 self.center = None

    	 if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		if radius > 2:
			cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			cv2.circle(frame, self.center, 5, (0, 0, 255), -1)



	 
         return frame

    
    def depth_callback(self, ros_image):
	try:
		depth_image = self.bridge.imgmsg_to_cv2(ros_image,"16UC1")
		print(depth_image.shape)
		
        except CvBridgeError, e:
            	print e
		
	depth_display_image = self.process_depth_image(depth_image)

	cv2.imshow("Depth Image", depth_image)	


    def process_depth_image(self, frame):
	u = self.center[1]
	v = self.center[0]
	depth = frame[u,v] # meters
	depth = depth / 1000.0
	#print(self.center, depth) 
	fx = 525.0
	fy = 525.0
	cx = 319.5
	cy = 239.5
        x = (depth / fx)*(v - cx)
      	y = (depth / fy)*(u - cy)
	z = depth
	print(x,y,z)
	br = tf.TransformBroadcaster()
	br.sendTransform((x,y,z), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), "box", "kinect_depthSensor")
	
	return frame


    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
   
 
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
