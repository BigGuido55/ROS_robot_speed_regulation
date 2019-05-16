#!/usr/bin/python
import rospy
import tf2_ros
import geometry_msgs.msg
import sys

if __name__ == '__main__':
	rospy.init_node('final')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		try:
			tr = tfBuffer.lookup_transform(sys.argv[1], sys.argv[2], rospy.Time())
			print tr

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			continue
		
		rate.sleep()
