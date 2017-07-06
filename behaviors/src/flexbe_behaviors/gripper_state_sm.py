#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.IK_Solver import IKSolver
from flexbe_states.GripperCheck import GripperStateEffort
from flexbe_states.GripperWidth import GripperStateWidth
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 27 2017
@author: Alessio
'''
class Gripper_StateSM(Behavior):
	'''
	Ciao
	'''


	def __init__(self):
		super(Gripper_StateSM, self).__init__()
		self.name = 'Gripper_State'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		width = 0.004
		th = 5
		time = 100
		hertz = 100
		samples = 1000
		offset = 35
		wait = 2
		width_open = 0.0115
		target_pose = [-0.05, 0.0, 0.13]
		inclination_ee = 1.57
		rotation_ee = 0
		# x:30 y:365, x:599 y:17
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:155 y:77
			OperatableStateMachine.add('IK',
										IKSolver(target_pose=target_pose, inclination_ee=inclination_ee, rotation_ee=rotation_ee),
										transitions={'found': 'Open', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off})

			# x:401 y:170
			OperatableStateMachine.add('Check',
										GripperStateEffort(width=width, threshold=th, time=time),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:273 y:117
			OperatableStateMachine.add('Open',
										GripperStateWidth(width=width_open),
										transitions={'done': 'Check', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
