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
		self.distance = data.pose
		#print self.distance

	def calculate_distance(self, position):
		return math.sqrt(math.pow(position.x, 2) + math.pow(position.y, 2))

	#TODO create a proper apply_velocity function
	def apply_velocity(self, data):
		orientation = self.distance.orientation
		quat_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(quat_list)
		#print yaw
		
		new_distance = Point()
		new_distance.x = self.distance.position.x + self.apply_vel_coef * data.linear.x * math.cos(yaw)
		new_distance.y = self.distance.position.y + self.apply_vel_coef * data.linear.x * math.sin(yaw)
		return new_distance

	def callback(self, data):
		print data
		factor = 0.0
		cur_distance = self.calculate_distance(self.distance.position)
		print ("distance: " + str(cur_distance))
		if cur_distance < 0.25: factor = 1.3
		elif 0.25 <= cur_distance <= 1.0: factor = (3 - cur_distance) / 2
		elif 1.0 <= cur_distance <= 2: factor = 2 - cur_distance

		if self.calculate_distance(self.apply_velocity(data)) < cur_distance: factor = self.max_factor - factor

		data.linear.x *= factor
		data.linear.y *= factor
		data.linear.z *= factor
		if factor != 0:
			data.angular.x *= factor
			data.angular.y *= factor
			data.angular.z *= factor
		print ("factor: " + str(factor))
		self.pub.publish(data)

	def __init__(self):
		self.apply_vel_coef = 0.1
		self.max_factor = 1.3
		self.pub = rospy.Publisher('pioneer/cmd_vel', Twist, queue_size=1)
		self.distance = Pose()
		self.time = rospy.Time.now()
		rospy.Subscriber('cmd_vel', Twist, self.callback)
		rospy.Subscriber('relative_pose', PoseStamped, self.write_pose)

if __name__ == '__main__':
	rospy.init_node("vel_refac")
	try:
		node = Node()
		while not rospy.is_shutdown():
			rospy.sleep(1.0)
	except rospy.ROSInterruptException:
		pass
		
