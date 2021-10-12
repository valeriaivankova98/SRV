#! /usr/bin/python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math

class TurtleSubscriber:
	def __init__(self):
		rospy.Subscriber('/turtle1/pose', Pose, self.callback_turtle)
		rospy.Subscriber('/leo/pose', Pose, self.callback_leo)
		self.coords = { 'x': 0, 'y': 0}
		self.theta = 0
		self.publisher = rospy.Publisher('/leo/cmd_vel', Twist, queue_size = 1)

	def callback_turtle(self, msg):
		local_msg = Twist()
		if self.get_distance(msg.x, self.coords.get('x'), msg.y, self.coords.get('y')) < 0.2:
			return
		
		local_msg.linear.x = self.get_linear(msg.x, self.coords.get('x'), msg.y, self.coords.get('y')) / 10
		local_msg.angular.z = self.get_angular(msg.x, self.coords.get('x'), msg.y, self.coords.get('y'), self.theta)
		self.publisher.publish(local_msg)

	def callback_leo(self, msg):
		self.coords['x'] = msg.x
		self.coords['y'] = msg.y
		self.theta = msg.theta

	def get_linear(self, x1, x2, y1, y2):
		return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))

	def get_angular(self, x1, x2, y1, y2, theta):
		return math.atan2(y1 - y2, x1 - x2) - theta

	def get_distance(self, x1, x2, y1, y2):
		return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

rospy.init_node('lab0')
TurtleSubscriber()
rospy.spin()