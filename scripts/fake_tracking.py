#!/usr/bin/python
from gazebo_msgs.srv import GetModelState
import rospy
import tf2_ros
import geometry_msgs.msg
import sys

if __name__ == '__main__':
	rospy.init_node('actor_node')
	rate = rospy.Rate(100)
	global_frame = rospy.get_param("~global_frame")
	object_name = rospy.get_param("~object_name")
	while True:	
		try:
			broad = tf2_ros.TransformBroadcaster()
			model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			coordinates = model(object_name, '')		

			t = geometry_msgs.msg.TransformStamped()
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = global_frame				
			t.child_frame_id = object_name				
			t.transform.translation.x = coordinates.pose.position.x
			t.transform.translation.y = coordinates.pose.position.y
			t.transform.translation.z = coordinates.pose.position.z
			t.transform.rotation.x = coordinates.pose.orientation.x
			t.transform.rotation.y = coordinates.pose.orientation.y
			t.transform.rotation.z = coordinates.pose.orientation.z
			t.transform.rotation.w = coordinates.pose.orientation.w

			broad.sendTransform(t)

		except rospy.ServiceException as e:
			rospy.loginfo("Couldn't get model state: {0}".format(e))
		rate.sleep()


