#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.JointValues import JointValuePub
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 21 2015
@author: Philipp Schillinger
'''
class ExampleBehaviorSM(Behavior):
	'''
	This is a simple example for a behavior.
	'''


	def __init__(self):
		super(ExampleBehaviorSM, self).__init__()
		self.name = 'Example Behavior'

		# parameters of this behavior
		self.add_parameter('waiting_time', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 791 85 
		# This transition will only be executed if the Autonomy Level is greater than Low during execution, e.g. High



	def create(self):
		target = [2.95, 1.0, -1.0, 3.3, 2.95]
		# x:83 y:390
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:179 y:114
			OperatableStateMachine.add('gggg',
										JointValuePub(target_pose=target),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
