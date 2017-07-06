#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.JointValues import JointValuePub
from flexbe_behaviors.random_positions_sm import RandompositionsSM
from flexbe_behaviors.youbot_arm_manipulation_sm import YoubotArmManipulationSM
from flexbe_states.publisher_pose2D import PublisherPose2D
from flexbe_states.move_base_state import MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Apr 08 2017
@author: Matteo Pantano
'''
class YoubotCompleteTaskSM(Behavior):
	'''
	YouBot complete task, from searching cube to disposing it on its home position
	'''


	def __init__(self):
		super(YoubotCompleteTaskSM, self).__init__()
		self.name = 'Youbot Complete Task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(RandompositionsSM, 'Random positions')
		self.add_behavior(YoubotArmManipulationSM, 'Youbot Arm Manipulation')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pose_cube = "/box_pose_footprint"
		inclination = 1.57
		height_above = 0
		height_grasp = 30
		j_search = [2.95, 1.5, -1.6, 3.5, 2.95]
		width_open = 0.0115
		j_ground = [2.95, 2.35, -1.6, 2.6, 2.95]
		hertz = 100
		samples = 1000
		offset = 35
		th = 5
		time = 1500
		width = 0.004
		wait = 2
		target_pose = [-0.05, 0.0, 0.14]
		rotation_ee = 0
		target_pose_above = [-0.025, 0.0, 0.155]
		inclination_ee = 1.57
		home_base = [0.0, 0.0, 0.0]
		j_start = [2.95, 1.0, -1.0, 3.3, 2.95]
		# x:51 y:467, x:48 y:357
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:79 y:27
			OperatableStateMachine.add('Search(1)',
										JointValuePub(target_pose=j_start),
										transitions={'done': 'Random positions', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:240 y:20
			OperatableStateMachine.add('Random positions',
										self.use_behavior(RandompositionsSM, 'Random positions'),
										transitions={'finished': 'Youbot Arm Manipulation', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:486 y:18
			OperatableStateMachine.add('Youbot Arm Manipulation',
										self.use_behavior(YoubotArmManipulationSM, 'Youbot Arm Manipulation'),
										transitions={'finished': 'Move base home', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:514 y:160
			OperatableStateMachine.add('Move base home',
										PublisherPose2D(target_pose=home_base),
										transitions={'done': 'Move Base server'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'move_base_home'})

			# x:511 y:269
			OperatableStateMachine.add('Move Base server',
										MoveBaseState(),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'move_base_home'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
