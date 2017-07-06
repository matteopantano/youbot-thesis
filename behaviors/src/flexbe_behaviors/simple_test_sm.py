#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.GripperWidth import GripperStateWidth
from flexbe_states.JointValues import JointValuePub
from flexbe_states.publish_pose_state import PublishPoseState
from flexbe_states.JointValuesIK import JointValueIK
from flexbe_states.JointPub4IK import JointPubIK
from flexbe_states.publish_pose_state_IK import PublishPoseStateIK
from flexbe_states.move_base_footprint_state import MoveBaseFootprintState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D
# [/MANUAL_IMPORT]


'''
Created on Tue Nov 17 2015
@author: Spyros Maniatopoulos
'''
class SimpleTestSM(Behavior):
	'''
	Simple behavior for the purposes of testing FlexBE - youBot integration.
	'''


	def __init__(self):
		super(SimpleTestSM, self).__init__()
		self.name = 'Simple Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		traj_ik = [2.95, 1.1, -1.6, 3.5, 2.95]
		traj_search = [2.95, 1.0, -1.0, 2.4, 2.95]
		box_pose = "/box_pose"
		box_pose_IK = "/box_pose_ik"
		above_height = 0.02
		on_height = 0.2
		close = 0.004
		open = 0.0115
		angle = 1.57
		# x:38 y:610, x:27 y:398
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

        # [/MANUAL_CREATE]


		with _state_machine:
			# x:43 y:65
			OperatableStateMachine.add('OpeningGripper',
										GripperStateWidth(width=open),
										transitions={'done': 'JointSearch', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:752 y:58
			OperatableStateMachine.add('JointIK',
										JointValuePub(target_pose=traj_ik),
										transitions={'done': 'PublishPose4IK', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:331 y:64
			OperatableStateMachine.add('PublishPose',
										PublishPoseState(topic=box_pose_IK, theta=0.0),
										transitions={'received': 'Move_Base(1)', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Low, 'unavailable': Autonomy.Low},
										remapping={'output_value': 'output_value_pose', 'message': 'message'})

			# x:844 y:191
			OperatableStateMachine.add('GetJointIK',
										JointValueIK(above=above_height, grasp_angle=angle),
										transitions={'found': 'AboveGrasp', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'pose': 'output_value_ik', 'output_ik': 'output_ik'})

			# x:841 y:305
			OperatableStateMachine.add('AboveGrasp',
										JointPubIK(),
										transitions={'done': 'GetJointIKcube', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'output_ik'})

			# x:837 y:399
			OperatableStateMachine.add('GetJointIKcube',
										JointValueIK(above=on_height, grasp_angle=angle),
										transitions={'found': 'OnGrasp', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'pose': 'output_value_ik', 'output_ik': 'output_ik_on'})

			# x:852 y:515
			OperatableStateMachine.add('OnGrasp',
										JointPubIK(),
										transitions={'done': 'ClosureGripper', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'output_ik_on'})

			# x:849 y:665
			OperatableStateMachine.add('JointHome',
										JointValuePub(target_pose=traj_ik),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:838 y:585
			OperatableStateMachine.add('ClosureGripper',
										GripperStateWidth(width=close),
										transitions={'done': 'JointHome', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:191 y:64
			OperatableStateMachine.add('JointSearch',
										JointValuePub(target_pose=traj_search),
										transitions={'done': 'PublishPose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:918 y:66
			OperatableStateMachine.add('PublishPose4IK',
										PublishPoseStateIK(topic=box_pose_IK),
										transitions={'received': 'GetJointIK', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'output_value': 'output_value_ik', 'message': 'message'})

			# x:512 y:60
			OperatableStateMachine.add('Move_Base(1)',
										MoveBaseFootprintState(),
										transitions={'arrived': 'JointIK', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'output_value_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

    # [/MANUAL_FUNC]
