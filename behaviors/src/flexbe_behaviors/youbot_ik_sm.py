#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.JointValues import JointValuePub
from flexbe_states.publish_pose_state_IK import PublishPoseStateIK
from flexbe_states.IK_Solver import IKSolver
from flexbe_states.GripperWidth import GripperStateWidth
from flexbe_states.IK_Solver_Trajectory import IKSolverTrajectory
from flexbe_states.GripperCheck import GripperStateEffort
from flexbe_behaviors.test_sm import TestSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Apr 08 2017
@author: Alessio Caporali
'''
class YoubotIKSM(Behavior):
	'''
	Invese Kinematics State Machine for the Kuka Youbot
	'''


	def __init__(self):
		super(YoubotIKSM, self).__init__()
		self.name = 'Youbot IK'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(TestSM, 'Test')

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
		# x:51 y:467, x:48 y:357
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:43 y:32
			OperatableStateMachine.add('Search(1)',
										JointValuePub(target_pose=j_search),
										transitions={'done': 'Test', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:616 y:23
			OperatableStateMachine.add('pose',
										PublishPoseStateIK(topic=pose_cube),
										transitions={'received': 'Trajectory', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'output_value': 'output_value', 'message': 'message'})

			# x:1022 y:137
			OperatableStateMachine.add('IK_Plate',
										IKSolver(target_pose=target_pose, inclination_ee=inclination_ee, rotation_ee=rotation_ee),
										transitions={'found': 'Gripper_Plate_Release', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off})

			# x:993 y:295
			OperatableStateMachine.add('Gripper_Plate_Release',
										GripperStateWidth(width=width_open),
										transitions={'done': 'IK_Plate_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1025 y:399
			OperatableStateMachine.add('IK_Plate_2',
										IKSolver(target_pose=target_pose_above, inclination_ee=inclination_ee, rotation_ee=rotation_ee),
										transitions={'found': 'Home', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off})

			# x:809 y:24
			OperatableStateMachine.add('Trajectory',
										IKSolverTrajectory(hertz=hertz, samples=samples, offset=offset, inclination_ee=inclination_ee),
										transitions={'found': 'Check', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'pose': 'output_value'})

			# x:981 y:25
			OperatableStateMachine.add('Check',
										GripperStateEffort(width=width, threshold=th, time=time),
										transitions={'done': 'IK_Plate', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:453 y:25
			OperatableStateMachine.add('Open',
										GripperStateWidth(width=width_open),
										transitions={'done': 'pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1024 y:482
			OperatableStateMachine.add('Home',
										JointValuePub(target_pose=j_search),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:308 y:27
			OperatableStateMachine.add('Search',
										JointValuePub(target_pose=j_search),
										transitions={'done': 'Open', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:167 y:28
			OperatableStateMachine.add('Test',
										self.use_behavior(TestSM, 'Test'),
										transitions={'finished': 'Search', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
