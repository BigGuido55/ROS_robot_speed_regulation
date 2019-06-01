#!/usr/bin/python
import rospy
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class Node():
	def write_pose(self, data):
		self.pose = data.pose

	def calculate_distance(self, position):
		return math.sqrt(math.pow(position.x, 2) + math.pow(position.y, 2))

	def apply_velocity(self, data):
		orientation = self.pose.orientation
		quat_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(quat_list)
		
		new_distance = Point()
		new_distance.x = self.pose.position.x + self.time_delta * data.linear.x * math.cos(yaw)
		new_distance.y = self.pose.position.y + self.time_delta * data.linear.x * math.sin(yaw)
		return new_distance

	def callback(self, data):
		#print data
		max_vel_factor = rospy.get_param("~max_vel_factor")
		ideal_distance = rospy.get_param("~ideal_distance")
		self.time_delta = rospy.get_param("~time_delta")

		vel_factor = 0.0
		cur_distance = self.calculate_distance(self.pose.position)
		print ("distance: " + str(cur_distance))

		if self.calculate_distance(self.apply_velocity(data)) < cur_distance:
			if (0.4 * ideal_distance) <= cur_distance <= ideal_distance:
				vel_factor = (5 * cur_distance / ideal_distance - 2) / 3
			elif ideal_distance <= cur_distance <= 2 * ideal_distance:
				vel_factor = (max_vel_factor - 1) * (cur_distance / ideal_distance - 1) + 1
			elif cur_distance >= 2 * ideal_distance:
				vel_factor = max_vel_factor
		else:
			if cur_distance < (0.4 * ideal_distance): 
				vel_factor = max_vel_factor
			elif (0.4 * ideal_distance) <= cur_distance <= ideal_distance: 
				vel_factor = (5 * cur_distance * (1 - max_vel_factor) / ideal_distance - 2 + 5 * max_vel_factor) / 3
			elif ideal_distance <= cur_distance <= 2 * ideal_distance: 
				vel_factor = 2 - cur_distance / ideal_distance

		data.linear.x *= vel_factor
		data.linear.y *= vel_factor
		data.linear.z *= vel_factor
		if vel_factor != 0:
			data.angular.x *= vel_factor
			data.angular.y *= vel_factor
			data.angular.z *= vel_factor
		print ("factor: " + str(vel_factor) + "\n")
		self.pub.publish(data)

	def __init__(self):
		self.pub = rospy.Publisher('pioneer/cmd_vel', Twist, queue_size=1)
		self.pose = Pose()
		rospy.Subscriber('cmd_vel', Twist, self.callback)
		rospy.Subscriber('relative_pose', PoseStamped, self.write_pose)

if __name__ == '__main__':
	rospy.init_node("vel_adaptation")
	try:
		node = Node()
		while not rospy.is_shutdown():
			rospy.sleep(1.0)
	except rospy.ROSInterruptException:
		pass
		
