#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry


class Map2OdomPublisher:
	def __init__(self):
		self.broadcaster = tf.TransformBroadcaster()
		self.subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.callback)

	def callback(self, odom_msg):
		self.odom_msg = odom_msg

	def spin(self):
		
		self.broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'odom', 'map')


		'''pose = self.odom_msg.pose.pose
		pos = (pose.position.x, pose.position.y, pose.position.z)
		quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

		map_frame_id = "map"
		odom_frame_id = 'odom'

		#self.broadcaster.sendTransform(pos, quat, rospy.Time.now(), odom_frame_id, map_frame_id)
        '''

def main():
	rospy.init_node('map2odom_publisher')
	node = Map2OdomPublisher()

	while not rospy.is_shutdown():
		node.spin()


if __name__ == '__main__':
	main()