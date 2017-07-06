#!/usr/bin/env python

import rospy
import rostopic
import inspect
from std_msgs.msg import Empty
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D, Pose
# [/MANUAL_IMPORT]

'''
Created on 30.03.2017

@author: Matteo Pantano
'''

class PublishPoseStateIK(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 		Topic to which the pose will be read.

	#> output_value		object		The 2D pose message.
	#> message		object		Latest message on the given topic of the respective type.

	<= received 				Message has been received and stored in userdata.
	<= unavailable 				The topic is not available when this state becomes actives.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(PublishPoseStateIK, self).__init__(outcomes=['received', 'unavailable'],output_keys=['output_value','message'])
		
		self._topic = topic
		self._connected = False
		
		(msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)

		Logger.logwarn('QUI:\nFound: %s' % (str(msg_topic)))


		if msg_topic == self._topic:
			msg_type = self._get_msg_from_path(msg_path)
			self._sub = ProxySubscriberCached({self._topic: msg_type})
			self._connected = True
		else:
			Logger.logwarn('Topic %s for state %s not yet available.\nFound: %s\nWill try again when entering the state...' % (self._topic, self.name, str(msg_topic)))


	def execute(self, userdata):
		'''
		Execute this state
		'''
		if not self._connected:
			return 'unavailable'

		if self._sub.has_msg(self._topic):
			userdata.message = self._sub.get_last_msg(self._topic)
			# assign data inside to the topic to local variable 
			data = self._sub.get_last_msg(self._topic)
			#print data.position.x
			# go to pose of cube
			target_pose = Pose(data.position, data.orientation)
			userdata.output_value = target_pose
			self._sub.remove_last_msg(self._topic)
			return 'received'

	
	def on_enter(self, userdata):
		if not self._connected:
			(msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
			if msg_topic == self._topic:
				msg_type = self._get_msg_from_path(msg_path)
				self._sub = ProxySubscriberCached({self._topic: msg_type})
				self._connected = True
				Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
			else:
				Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

		if self._connected and self._sub.has_msg(self._topic):
			self._sub.remove_last_msg(self._topic)
		


	def _get_msg_from_path(self, msg_path):
		msg_import = msg_path.split('/')
		msg_module = '%s.msg' % (msg_import[0])
		package = __import__(msg_module, fromlist=[msg_module])
		clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
		return clsmembers[0][1]

	def on_exit(self, userdata):
		Logger.loginfo('Exiting Pub Pose State IK')

