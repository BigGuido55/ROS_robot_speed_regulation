#!/usr/bin/python
import rospy
import tf2_ros
import geometry_msgs.msg
import sys
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
	rospy.init_node('final')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rate = rospy.Rate(100)
	object_name = rospy.get_param("~object_name")
	tracking_frame = rospy.get_param("~tracking_frame")

	pub = rospy.Publisher('relative_pose', PoseStamped, queue_size=1)
	pose = PoseStamped()

	while not rospy.is_shutdown():
		try:
			tr = tfBuffer.lookup_transform(object_name, tracking_frame, rospy.Time())
			pose.header = tr.header		#ne svida mi se to sto pise "animated_box" za frame
			pose.pose.position.x = tr.transform.translation.x
			pose.pose.position.y = tr.transform.translation.y
			pose.pose.position.z = tr.transform.translation.z
			pose.pose.orientation = tr.transform.rotation
			pub.publish(pose)

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			continue
	
		rate.sleep()
