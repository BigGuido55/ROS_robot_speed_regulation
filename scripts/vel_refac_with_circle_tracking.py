#!/usr/bin/python
import rospy
import math
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
		#print self.pioneerPose

	def write_robot_goal(self, data):
		self.pioneerGoalPose = data.pose
		#print self.pioneerGoalPose

	def write_circles(self, data):
		self.circles = data.circles
		#print self.circles
		#print "\n"

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
		for circle in self.circles:
			#print self.calculate_distance(self.actor.center, circle.center)
			if self.calculate_distance(self.actor.center, circle.center) < 0.2:
				self.actor = circle
				self.count = 0
				return
		self.count += 1
		print "no actor: " + str(self.count)

	#TODO create a proper apply_velocity function
	def apply_velocity(self, data):
		orientation = self.pioneerPose.orientation
		quat_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(quat_list)
		
		new_distance = Point()
		new_distance.x = self.pioneerPose.position.x + self.delta_time * data.linear.x * math.cos(yaw)
		new_distance.y = self.pioneerPose.position.y + self.delta_time * data.linear.x * math.sin(yaw)
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
		print "Do cilja: " + str(self.calculate_distance(self.pioneerPose.position, self.pioneerGoalPose.position))
		if self.calculate_distance(self.pioneerPose.position, self.pioneerGoalPose.position) < self.acceptableDistance:	#TELLS ROBOT TO STOP SPINNING, NEEDS REFINING
			self.stop_robot()
			self.actor = None
			return

		max_vel_factor = rospy.get_param("~max_vel_factor")
		ideal_distance = rospy.get_param("~ideal_distance")

		if self.actor == None:
			if not self.find_actor():
				print "No available actors!!!"
				return
		else:
			self.update_actor()
			if self.count > 100:
				self.stop_robot()
				rospy.signal_shutdown("Actor has been lost!!!")

		vel_factor = 0.0
		cur_distance = self.calculate_distance(self.pioneerPose.position, self.actor.center)
		print ("distance: " + str(cur_distance))

		if self.calculate_distance(self.apply_velocity(data), self.actor.center) < cur_distance:
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
		print data
		print ("factor: " + str(vel_factor) + "\n")
		self.pub.publish(data)

	def __init__(self):
		self.delta_time = 0.01
		self.acceptableDistance = 0.4			#FOR STOPPING AT THE GOAL
		self.pub = rospy.Publisher('pioneer/cmd_vel', Twist, queue_size=1)
		self.pioneerPose = Pose()
		self.count = 0					#IF REACHES 250, WE LOST ACTOR
		self.actor = None
		self.circles = None
		rospy.Subscriber('cmd_vel', Twist, self.callback)
		rospy.Subscriber('relative_pose', PoseStamped, self.write_robot_pose)
		rospy.Subscriber('move_base_simple/goal', PoseStamped, self.write_robot_goal)
		rospy.Subscriber('obstacles', Obstacles, self.write_circles)

if __name__ == '__main__':
	rospy.init_node("vel_refac_with_circle_tracking")
	try:
		node = Node()
		while not rospy.is_shutdown():
			rospy.sleep(1.0)
	except rospy.ROSInterruptException:
		pass
		
