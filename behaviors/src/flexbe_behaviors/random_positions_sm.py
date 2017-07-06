#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.publisher_pose2D import PublisherPose2D
from flexbe_states.move_base_state import MoveBaseState
from flexbe_states.publish_pose_state import PublishPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 20 2017
@author: Matteo Pantano
'''
class RandompositionsSM(Behavior):
	'''
	Behaviour for random search of the cube 
	'''


	def __init__(self):
		super(RandompositionsSM, self).__init__()
		self.name = 'Random positions'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		box_pose_IK = "/box_pose_ik"
		angle = 1.57
		pose_1 = [0.40, 0.0, 0.0]
		pose_2 = [0.60, 0.2, 0.0]
		# x:30 y:365, x:32 y:260
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['box_pose_container_1'])

		with _sm_container_0:
			# x:92 y:36
			OperatableStateMachine.add('Box_Pose(1)',
										PublishPoseState(topic=box_pose_IK, theta=0),
										transitions={'received': 'finished', 'unavailable': 'Box_Pose(1)'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'output_value': 'box_pose_container_1', 'message': 'message'})


		# x:30 y:365, x:29 y:254
		_sm_container_2_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pose_1_map'])

		with _sm_container_2_1:
			# x:73 y:34
			OperatableStateMachine.add('Move_Base(1)',
										MoveBaseState(),
										transitions={'arrived': 'Pose(2)', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose_1_map'})

			# x:215 y:36
			OperatableStateMachine.add('Pose(2)',
										PublisherPose2D(target_pose=pose_2),
										transitions={'done': 'Move_Base(2)'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'pose_2_map'})

			# x:379 y:33
			OperatableStateMachine.add('Move_Base(2)',
										MoveBaseState(),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose_2_map'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_container_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['pose_1_map'], output_keys=['box_pose_container_1'], conditions=[
										('finished', [('Container', 'finished'), ('Container_2', 'finished')]),
										('failed', [('Container_2', 'failed'), ('Container', 'failed')])
										])

		with _sm_container_2:
			# x:99 y:47
			OperatableStateMachine.add('Container_2',
										_sm_container_2_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose_1_map': 'pose_1_map'})

			# x:309 y:46
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'box_pose_container_1': 'box_pose_container_1'})



		with _state_machine:
			# x:87 y:30
			OperatableStateMachine.add('Pose(1)',
										PublisherPose2D(target_pose=pose_1),
										transitions={'done': 'Container'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'pose_1_map'})

			# x:237 y:25
			OperatableStateMachine.add('Container',
										_sm_container_2,
										transitions={'finished': 'Move_Base_BOX', 'failed': 'Container'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose_1_map': 'pose_1_map', 'box_pose_container_1': 'box_pose_container_1'})

			# x:241 y:260
			OperatableStateMachine.add('Move_Base_BOX',
										MoveBaseState(),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'box_pose_container_1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
