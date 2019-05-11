#!/usr/bin/python
import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
	rospy.init_node('proba')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('pioneer/odom', 'pioneer/base_link', rospy.Time())
			broad = tf2_ros.TransformBroadcaster()
			
			t = geometry_msgs.msg.TransformStamped()
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = "ground_zero"
			t.child_frame_id = "pioneer/base_link"
			t.transform.translation.x = trans.transform.translation.x
			t.transform.translation.y = trans.transform.translation.y
			t.transform.translation.z = trans.transform.translation.z
			t.transform.rotation.x = trans.transform.rotation.x
			t.transform.rotation.y = trans.transform.rotation.y
			t.transform.rotation.z = trans.transform.rotation.z
			t.transform.rotation.w = trans.transform.rotation.w

			broad.sendTransform(t)



			#print str(trans.transform.translation.x) + ' ' + str(trans.transform.translation.y)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			continue
		
		rate.sleep()
	
	
	
	
	
	
	
