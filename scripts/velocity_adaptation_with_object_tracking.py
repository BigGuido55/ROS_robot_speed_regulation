#!/usr/bin/python
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle

class Node():
	def write_robot_pose(self, data):
		self.pioneerPose = data.pose


	def write_robot_goal(self, data):
		self.pioneerGoalPose = data.pose
		self.actor = None


	def write_circles(self, data):
		object_frame = rospy.get_param("~object_frame")
		global_frame = rospy.get_param("~global_frame")									#NE MOZE MAPA BITI OBJECT...
		tf = self.tfBuffer.lookup_transform(object_frame, global_frame, rospy.Time())
		
		orientation = tf.transform.rotation
		quat_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(quat_list)

		#newData = Obstacles()						#for publishing transformed circles
		#newData.circles = []
		#newData.header = data.header
		#newData.header.frame_id = "pioneer/map"

		self.circles = []
		for circle in data.circles:
			x = self.pioneerPose.position.x + math.cos(yaw) * circle.center.x + math.sin(yaw) * circle.center.y
			y = self.pioneerPose.position.y - math.sin(yaw) * circle.center.x + math.cos(yaw) * circle.center.y
			circle.center.x = x
			circle.center.y = y

			self.circles.append(circle)
			#newData.circles.append(circle)

		#self.IDEMO.publish(newData)

		max_num_of_no_actor = rospy.get_param("~max_num_of_no_actor")
		self.header = data.header
		self.header.frame_id = global_frame
		if self.actor != None:
			self.update_actor()
			if self.count >= max_num_of_no_actor:
				self.stop_robot()
				rospy.signal_shutdown("Actor has been lost!!!")


	def calculate_distance(self, firstPosition, secondPosition):
		return math.sqrt(math.pow((firstPosition.x - secondPosition.x), 2) + math.pow((firstPosition.y - secondPosition.y), 2))


	def find_actor(self):
		min_distance = None

		if self.circles == None: 
			return False

		for circle in self.circles:
			distance = self.calculate_distance(self.pioneerPose.position, circle.center)
			if min_distance == None:
				min_distance = distance
				self.actor = circle
			elif min_distance > distance:
				min_distance = distance
				self.actor = circle

		if self.actor == None:
			return False
		else:
			return True


	def update_actor(self):
		same_object_distance = rospy.get_param("~same_object_distance")

		for circle in self.circles:
			if self.calculate_distance(self.actor.center, circle.center) < same_object_distance:
				self.actor = circle
				self.count = 0
				return
		self.count += 1
		print "no actor: " + str(self.count)


	def apply_velocity(self, data):
		time_delta = rospy.get_param("~time_delta")
		orientation = self.pioneerPose.orientation
		quat_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(quat_list)
		
		new_distance = Point()

		if data.angular.z != 0.0:
			radius = data.linear.x / data.angular.z
			angle = data.angular.z * time_delta

			new_distance.x = self.pioneerPose.position.x - radius * math.sin(yaw) + radius * math.sin(yaw + angle)
			new_distance.y = self.pioneerPose.position.y + radius * math.cos(yaw) - radius * math.cos(yaw + angle)	
		else:
			new_distance.x = self.pioneerPose.position.x + time_delta * data.linear.x * math.cos(yaw)
			new_distance.y = self.pioneerPose.position.y + time_delta * data.linear.x * math.sin(yaw)
		return new_distance


	def stop_robot(self):
		data = Twist()
		data.linear.x = 0
		data.linear.y = 0
		data.linear.z = 0
		data.angular.x = 0
		data.angular.y = 0
		data.angular.z = 0
		self.pub.publish(data)


	def callback(self, data):
		if self.actor == None:
			if not self.find_actor():
				print "No available actors!!!"
				return

		max_vel_factor = rospy.get_param("~max_vel_factor")
		ideal_distance = rospy.get_param("~ideal_distance")
		LRB = rospy.get_param("~lower_relative_bound")
		URB = rospy.get_param("~upper_relative_bound")
		

		vel_factor = 0.0
		cur_distance = self.calculate_distance(self.pioneerPose.position, self.actor.center)
		print ("distance: " + str(cur_distance))

		if self.calculate_distance(self.apply_velocity(data), self.actor.center) < cur_distance:
			if (LRB * ideal_distance) <= cur_distance <= ideal_distance:
				vel_factor = (cur_distance / ideal_distance - LRB) / (1 - LRB)								
			elif ideal_distance <= cur_distance <= 2 * ideal_distance:
				vel_factor = ((max_vel_factor - 1) * cur_distance / ideal_distance + URB - max_vel_factor) / (URB - 1)
			elif cur_distance >= URB * ideal_distance:
				vel_factor = max_vel_factor
		else:
			if cur_distance < (LRB * ideal_distance):
				vel_factor = max_vel_factor
			elif (LRB * ideal_distance) <= cur_distance <= ideal_distance:
				vel_factor = ((1 - max_vel_factor) * cur_distance / ideal_distance + max_vel_factor - LRB) / (1 - LRB)
			elif ideal_distance <= cur_distance <= URB * ideal_distance:
				vel_factor = -(cur_distance / ideal_distance - URB) / (URB - 1)

		data.linear.x *= vel_factor
		data.linear.y *= vel_factor
		data.linear.z *= vel_factor
		if vel_factor != 0:
			data.angular.x *= vel_factor
			data.angular.y *= vel_factor
			data.angular.z *= vel_factor
		print ("factor: " + str(vel_factor) + "\n")
		self.pub.publish(data)

		obs = Obstacles()
		obs.header = self.header
		obs.circles.append(self.actor)
		self.indicator.publish(obs)


	def __init__(self):
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.pub = rospy.Publisher(rospy.get_param("~velocity_topic"), Twist, queue_size=1)
		self.indicator = rospy.Publisher("actor_position", Obstacles, queue_size=1)
		#self.IDEMO = rospy.Publisher("IDEMO", Obstacles, queue_size=1)
		self.pioneerPose = Pose()
		self.count = 0
		self.actor = None
		self.circles = None
		rospy.Subscriber('cmd_vel', Twist, self.callback, queue_size=1)
		rospy.Subscriber('relative_pose', PoseStamped, self.write_robot_pose, queue_size=1)
		rospy.Subscriber('move_base_simple/goal', PoseStamped, self.write_robot_goal, queue_size=1)
		rospy.Subscriber('raw_obstacles', Obstacles, self.write_circles, queue_size=1)


if __name__ == '__main__':
	rospy.init_node("vel_adapt_with_object_tracking")
	try:
		node = Node()
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass
		
