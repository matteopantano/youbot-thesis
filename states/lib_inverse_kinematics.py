#!/usr/bin/env python

import rospy
import sys
import copy
import numpy as np
import math
import time
import logging
import std_msgs.msg
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion


def Inverse_Kinematics(position, inclination_link, rotation_cube):
	
		link = [0, 33, 155, 135, 176] #links lenght

		mode_reversed = 0 # equal to 1 if not (-135 < teta_1 < 135)
		
		pose_x = position[0] * 1000
		pose_y = position[1] * 1000
		pose_z = position[2] * 1000
		gamma = inclination_link

		if rotation_cube < -1.57:
			teta_5 = rotation_cube + 1.57
		if rotation_cube > 1.57:
			teta_5 = rotation_cube - 1.57
		else:
		 	teta_5 = rotation_cube

	
	
		#ANGLE JOINT_1 CALCULATION
		zeta = math.atan2(pose_y,(pose_x - 167))
		
		if (zeta > 2.35) or (zeta < -2.35):
			zeta = - (3.14 - zeta)
			mode_reversed = 1
			#print "_________ MODE_REVERSED ON _____________"

		link_2_correction = [(link[1] * math.cos(-zeta)), (link[1] * math.sin(-zeta))]



		# RRR JOINT 2-3-4 CALCULATION
		distance_target_link_2 = math.sqrt((pose_x - 167 - link_2_correction[0])**2 + (pose_y + link_2_correction[1])**2)
		if mode_reversed == 1:
			distance_target_link_2 = - distance_target_link_2
	
		target_height = - 200 + (pose_z) 
		fi = gamma


		vector_wrist = [(distance_target_link_2 - link[4]*math.cos(fi)), (target_height - link[4]*math.sin(fi))]
		distance_wrist = math.sqrt(vector_wrist[0]**2 + vector_wrist[1]**2)

		#teorema dei coseni, angolo joint 3
		cos_teta3 = (distance_wrist**2 - link[2]**2 - link[3]**2)/(2*link[2]*link[3])
		sin_teta3 = + math.sqrt(1 - cos_teta3**2)
		teta_3 =  math.atan2(sin_teta3,cos_teta3)


		cos_teta2 = link[2] + link[3] * cos_teta3
		sin_teta2 = link[3] * sin_teta3

		#teta2_in = math.atan2(sin_teta2, cos_teta2)
		#teta2_out = math.atan2(vector_wrist[1], vectort_wrist[0])

		teta_2 = math.atan2(vector_wrist[1], vector_wrist[0]) - math.atan2(sin_teta2, cos_teta2)
		#print("angolo joint 2", teta_2, math.degrees(teta_2))

		teta_4 = fi - (teta_2 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)
		#print("angolo joint 4", teta_4, math.degrees(teta_4))
		if mode_reversed == 1:		
			teta_4 = 6.28 + teta_4
			#print("teta_4 modificato", teta_4, math.degrees(teta_4))

		if ((teta_2 < 0) or (joint_4 > 3.4)) and (mode_reversed != 1):
			#print(" __________ NEGATIVE SINE MODE ON! _________ ")	
			sin_teta3 = - math.sqrt(1 - cos_teta3**2)
			teta_3 =  math.atan2(sin_teta3,cos_teta3)
			cos_teta2 = link[2] + link[3] * cos_teta3
			sin_teta2 = link[3] * sin_teta3
			teta_2 = math.atan2(vector_wrist[1], vector_wrist[0]) - math.atan2(sin_teta2, cos_teta2)
			teta_4 = + fi - (teta_2 + teta_3)
			


		if (mode_reversed == 1) and (teta_2 < -3.14):
			#print " _________ ANGOLO ESTREMO __________ teta_2: ", teta_2
			teta_2 = 6.28 + teta_2 
			#teta_4 = - fi - (teta_2 + teta_3)
			teta_4 = 6.28 - (-fi + (teta_2) + teta_3)

			#print(teta_2, teta_3, teta_4)
			



		teta_1 = zeta
		teta_tool = gamma

		#-----------------------------
		# MAPPING for Vrep
		joint_1 = 2.94961 - teta_1
		joint_2 = 2.70526 - teta_2
		joint_3 = 0.523599 - (3.14 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)
		joint_5 = 2.92 + teta_5 - zeta  #to compensate if x*y is different from zero

		#if mode_reversed == 1:
		#	joint_5 = 0

		if joint_5 < 0:
			joint_5 = joint_5 + 3.14
		if joint_5 > 4.71:
			joint_5 = joint_5 - 3.14
		
		joint_q = (joint_1, joint_2, joint_3, joint_4, joint_5)

		#print("JOINT_1: ", teta_1, math.degrees(teta_1))
		#print("JOINT_2: ", teta_2, math.degrees(teta_2))
		#print("JOINT_3: ", teta_3, math.degrees(teta_3))
		#print("JOINT_4: ", teta_4, math.degrees(teta_4))
		#print("position wrist ", vector_wrist)
		#print("target_heigth" , target_height)



		return joint_q
	
