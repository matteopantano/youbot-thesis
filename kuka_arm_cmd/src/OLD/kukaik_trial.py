#!/usr/bin/env python

import sys
import numpy
import numpy as np
import math

# distance to base_footprint
pose_x = 360
pose_y = 0.0

b = 218
c = 135
d = 155
e = 255 # - 30  se si vuole l'end effector più in alto del suolo
 
gamma = 1.4
teta_5 = 2.94 

#teta_1 calculation
zeta = math.atan2(pose_y,(pose_x - 167))


# triangle AED
a = math.sqrt((pose_x - 200)**2 + pose_y**2) 
f = math.sqrt(a**2 + b**2 -2*(a*b*math.cos(gamma)))


alfa = math.asin((b*math.sin(gamma))/f)
beta = 3.14 - alfa - gamma #beta = math.asin((a*math.sin(gamma))/f)
print(alfa, beta)

# triangle ADB
epsilon = 1.57 - alfa
g = math.sqrt(e**2 + f**2 -2*(e*f*math.cos(epsilon)))

omega = math.asin((f*math.sin(epsilon))/g)
fi = math.asin((e*math.sin(epsilon))/g)

print(epsilon, fi, omega)
#triangle BCD
sigma = math.acos((c**2 + g**2 - d**2)/(2*g*c))
psi = math.asin((c*math.sin(sigma))/d)
delta = 3.14 - sigma - psi
print(sigma, psi, delta)


teta_1 = zeta
teta_2 = omega + psi
teta_3 = delta
teta_4 = sigma + fi + beta
teta_tool = gamma


print(teta_1, teta_2, teta_3, teta_4, teta_tool)


# MAPPING for Vrep

joint_1 = 2.94961 - teta_1
joint_2 = 2.70526 - (teta_2 - 1.57)
joint_3 = 0.523599 - teta_3 
joint_4 = 4.930555 - teta_4
joint_5 = teta_5

print(joint_1, joint_2, joint_3, joint_4, joint_5)







