#!/usr/bin/env python


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Empty

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D
# [/MANUAL_IMPORT]

'''
Created on 30.03.2017

@author: Matteo Pantano & Alessio Caporali
'''


class PublisherPose2D(EventState):
    '''
	Publishes a 2D Pose message for tutorial.

	-- target_pose   		float[]         Target for move base.

	#> output_value object				The 2D pose message.

	<= done 					Done publishing.

	'''

    def __init__(self, target_pose):
        '''
	Constructor
	'''
        super(PublisherPose2D, self).__init__(outcomes=['done'], output_keys=['output_value'])

	self._target = target_pose


    def execute(self, userdata):
	'''Execute this state'''

	target_pose = Pose2D(self._target[0], self._target[1], self._target[2])

	userdata.output_value = target_pose

        return 'done'

    def on_enter(self, userdata):
        pass
