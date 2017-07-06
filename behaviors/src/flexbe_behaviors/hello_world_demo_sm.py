#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from flexbe_states.log_state import LogState
from flexbe_states.GripperOpen import GripperStateOpen
from flexbe_states.GripperClosed import GripperStateClosed
from flexbe_states.GetCubePose import GetCubePose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Mar 29 2017
@author: Matteo Pantano
'''
class HelloWorldDemoSM(Behavior):
	'''
	Behavior created in the FlexBE tutorial.
	'''


	def __init__(self):
		super(HelloWorldDemoSM, self).__init__()
		self.name = 'Hello World Demo'

		# parameters of this behavior
		self.add_parameter('waiting_time', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		hello = "Hello World!"
		# x:82 y:296, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Initial_Wait',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'cube'},
										autonomy={'done': Autonomy.Off})

			# x:523 y:182
			OperatableStateMachine.add('Print_Greeting',
										LogState(text=userdata.pose, severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})

			# x:241 y:41
			OperatableStateMachine.add('Gripper',
										GripperStateOpen(duration=1.0),
										transitions={'done': 'Gripper_Close', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:547 y:37
			OperatableStateMachine.add('Gripper_Close',
										GripperStateClosed(duration=1.0),
										transitions={'done': 'Print_Greeting', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:764 y:72
			OperatableStateMachine.add('cube',
										GetCubePose(),
										transitions={'succeeded': 'Print_Greeting'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
