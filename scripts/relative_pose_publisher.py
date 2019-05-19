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
	object_name = rospy.get_param("~object_name")
	tracking_frame = rospy.get_param("~tracking_frame")

	while not rospy.is_shutdown():
		try:
			tr = tfBuffer.lookup_transform(object_name, tracking_frame, rospy.Time())
			print tr

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			continue
	
		rate.sleep()
