#!/usr/bin/env python
# import roslib; roslib.load_manifest('teleop_arm_ds4')
import rospy
from sensor_msgs.msg import JointState, Joy
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
from geometry_msgs.msg import Twist
import sys
import copy
import numpy


class teleop:

    def __init__(self):
        rospy.init_node('teleop')
        rospy.Subscriber("joint_states", JointState, self.processJoints)
        rospy.Subscriber('/joy', Joy, self.processJoystick)
        self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
        self.velPub = rospy.Publisher('/arm_controller/velocity_command', JointVelocities, queue_size=10)
        self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
	self.basePub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.joyMsg = None

        self.crntJointStates = JointState()
        self.crntJointPositions = JointPositions()
        self.gripperPositions = JointPositions()
        self.angularStep = 0.07
        self.maxVel = 1.0
        self.gripStep = 0.001
        self.previous_vel_change = False

        self.jointBounds = [
            {'upper': 5.80, 'lower': 0.011},
            {'upper': 2.60, 'lower': 0.020},
            {'upper': -0.015708, 'lower': -5.02654},
            {'upper': 3.40, 'lower': 0.03},
            {'upper': 5.60, 'lower': 0.12},
            {'upper': 0.0113, 'lower': 0.0},
            {'upper': 0.0113, 'lower': 0.0},
        ]


    def processJoints(self, joint_states):
        armStatesReceived = joint_states.name[10] == 'arm_joint_1'


        if armStatesReceived and self.joyMsg is not None:
	  
            self.crntJointStates = copy.deepcopy(joint_states)
            for index in range(0, len(joint_states.position)-2):
                pos = JointValue()
                pos.joint_uri = "arm_joint_" + str(index + 1)
                pos.unit = "rad"
                pos.value = joint_states.position[index]
                self.crntJointPositions.positions.append(pos)
            pos = JointValue()
            pos.joint_uri = "gripper_finger_joint_r"
            pos.unit = "m"
            pos.value = 0
            self.gripperPositions.positions.append(pos)

            pos = JointValue()
            pos.joint_uri = "gripper_finger_joint_l"
            pos.unit = "m"
            pos.value = 0
            self.gripperPositions.positions.append(pos)
            
            # desiredPos = copy.deepcopy(self.crntJointPositions)
            desiredGripPos = copy.deepcopy(self.gripperPositions)

            self.joyMsg.axes = numpy.array(self.joyMsg.axes)	    
            left_axis = self.joyMsg.axes[[0, 1]]
            right_axis = self.joyMsg.axes[[2, 3]]
            l2_axis = self.joyMsg.axes[12] * -1
            r2_axis = self.joyMsg.axes[13] * -1
	    square = self.joyMsg.buttons[15]
	    x = self.joyMsg.buttons[14]
	    circle = self.joyMsg.buttons[13]
            triangle = self.joyMsg.buttons[12]
	    r1 = self.joyMsg.buttons[11]
	    l1 = self.joyMsg.buttons[10]
	    r2 = self.joyMsg.buttons[9]
	    l2 = self.joyMsg.buttons[8]
            left_dpad = self.joyMsg.axes[6] == 1
            right_dpad = self.joyMsg.axes[6] == -1
            up_dpad = self.joyMsg.axes[7] == 1
            down_dpad = self.joyMsg.axes[7] == -1

            if not self.previous_vel_change:
                if square:
                    self.maxVel += 0.25
                if circle:
                    self.maxVel -= 0.25
                if self.maxVel < 0.25:
                    self.maxVel = 0.25

            self.previous_vel_change = square or circle

            # object that will carry all desired velocities to each joint
            desiredVel = JointVelocities()
	    distance = Twist()

            for index in range(0, len(joint_states.position)-2):
                vel = JointValue()
                vel.joint_uri = "arm_joint_" + str(index + 1)
                vel.unit = "s^-1 rad"  # radians per second
                desiredVel.velocities.append(vel)

	    if l1:
		desiredVel.velocities[0].value -= left_axis[0] * self.maxVel
		desiredVel.velocities[1].value -= left_axis[1] * self.maxVel
		if not r1 and l1:
		    desiredVel.velocities[2].value -= right_axis[1] * self.maxVel
		elif l1:
		    desiredVel.velocities[3].value -= right_axis[1] * self.maxVel

		desiredVel.velocities[4].value -= right_axis[0] * self.maxVel * 2

		desiredGripPos = self.gripperPositions
		desiredGripPos.positions[0].value = 0.0
		desiredGripPos.positions[1].value = 0.0

		# to open press square, to close relase square
		if square and l1:
		    desiredGripPos.positions[0].value = 0.0113
		    desiredGripPos.positions[1].value = desiredGripPos.positions[0].value
	    else:
		distance.linear.x = left_axis[1] * 5
		distance.linear.y = left_axis[0] * 5
		distance.angular.z = right_axis[0] * 0.6	    

            joint_message = ''
            vel_message = ''
            grip_message = ''
            for jointNum in range(5):
                joint_message += ('joint ' + str(jointNum) + ': ' + str(self.crntJointPositions.positions[jointNum].value) + '    ')
                vel_message += ('veloc ' + str(jointNum) + ': ' + str(desiredVel.velocities[jointNum].value) + '    ')	    

	    pos = JointValue()
            pos.joint_uri = "gripper_finger_joint_r"
            pos.unit = "m"
            pos.value = desiredGripPos.positions[0].value
            self.gripperPositions.positions.append(pos)

	    pos = JointValue()
            pos.joint_uri = "gripper_finger_joint_l"
            pos.unit = "m"
            pos.value = desiredGripPos.positions[1].value
            self.gripperPositions.positions.append(pos)

            #print(joint_message)
            #print(vel_message)
	    #print("gripper_finger_joint_r:",desiredGripPos.positions[0].value)
	    #print("gripper_finger_joint_l:",desiredGripPos.positions[1].value)
		
	    self.basePub.publish(distance)
            self.gripPub.publish(self.gripperPositions)
            self.velPub.publish(desiredVel)

    def checkValues(self, val, jointNum):
        # print("Received checkValues:", val, jointNum)
        jointNum = int(jointNum)
        # Limit angles for the joints, list is not entirely correct

        if val > self.jointBounds[jointNum]['upper']:
            val = self.jointBounds[jointNum]['upper']
        elif val < self.jointBounds[jointNum]['lower']:
            val = self.jointBounds[jointNum]['lower']
        return val

    def processJoystick(self, joyMsg):
        self.joyMsg = joyMsg


def main(args):
    teleop()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
