#!/usr/bin/env python

import sys
import numpy
import ikpy 
import numpy as np
from ikpy import plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


my_chain = ikpy.chain.Chain.from_urdf_file("/home/matteo/rover_ws/urdf/youbot.urdf", base_elements=["arm_link_0", "arm_joint_1", "arm_link_1", "arm_joint_2", "arm_link_2", "arm_joint_3", "arm_link_3", "arm_joint_4", "arm_link_4", "arm_joint_5", "arm_link_5"])
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

target = [0.2, 0.0, 0.50]
frame_target = np.eye(4)
frame_target[:3, 3] = target

joints = [0] * len(my_chain.links)
ik = my_chain.inverse_kinematics(frame_target, initial_position=joints)

print("arm_joint_2: ", ik[1])
print("arm_joint_3: ", ik[2])
print("arm_joint_4: ", ik[3])
print("arm_joint_5: ", ik[4])
my_chain.plot(ik, ax, target=target)
matplotlib.pyplot.show()





