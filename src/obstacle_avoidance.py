#!/usr/bin/env python

import math, time
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

DISTANCE_STEER = 0.4
DISTANCE_BREAK = 0.2

K_FRONT_DIST_TO_SPEED = 1.0
K_LAT_DIST_TO_STEER = 2.0

TIME_KEEP_STEERING = 1.5

def clip(value, minimum, maximum):
	if value < minimum:
		return minimum
	elif value > maximum:
		return maximum
	return value

class ObstacleAvoidance():
	def __init__(self):
		self.range_center = 2
		self.range_left = 2
		self.range_right = 2

		self.subsriber_left   = rospy.Subscriber("/robot/ultrasonic/0", Range, self.update_range)
		self.subscriber_center = rospy.Subscriber("/robot/ultrasonic/1", Range, self.update_range)
		self.subscriber_right  = rospy.Subscriber("/robot/ultrasonic/2", Range, self.update_range)
		rospy.loginfo("Subscribers set")

		self.publisher_twist = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=10)
		rospy.loginfo("Publisher set")

		self._message = Twist()

		self._time_steer = 0
		self._steer_sign_prev = 0
    
	def update_range(self, message):
		angle = message.field_of_view

		if abs(angle) < 0.1:
			self.range_center = message.range

		elif  angle > 0:
			self.range_right = message.range

		elif angle < 0:
			self.range_left = message.range
    
	def get_control_action(self):
		break_action = 0.5
		steer_action = 0.0

		range_min = min([self.range_left, self.range_center, self.range_right])

		if self.range_center < DISTANCE_STEER:
			adim_distance = range_min/DISTANCE_STEER

			if range_min < DISTANCE_BREAK:
				break_action = K_FRONT_DIST_TO_SPEED*(range/DISTANCE_BREAK)
				break_action = clip(break_action, 0, 0.5)
				rospy.loginfo("Engaging break %.1f"%break_action)

			steer_action = K_LAT_DIST_TO_STEER*(1.0 - adim_dist)
			steer_action = self.get_signed_steer(steer_action)

			steer_action   = clip(steer_action, -1.0, 1.0)
			rospy.loginfo("Steering command %.2f"%steer_action)

		return (break_action, steer_action)
    
	def get_signed_steer(self, steer_action):
		if time.time() > self._time_steer + TIME_KEEP_STEERING:
			self._time_steer  = time.time()

			if self.range_left < self.range_right:
				steer_action = -steer_action

			if steer_action >= 0:
				self._steer_sign_prev = 1
			else:
				self._steer_sign_prev = -1

		else:
			steer_action *= self._steer_sign_prev

		return (steer_action)

	def run(self):
		rate = rospy.Rate(5)

		while not rospy.is_shutdown():
			break_action, steer_action = self.get_control_action()

			self._message.linear.x = break_action
			self._message.angular.z = steer_action

			self.publisher_twist.publish(self._message)

			rate.sleep()

def main():
	rospy.init_node('obstacle_avoidance')
	obstacle_avoidance = ObstacleAvoidance()
	ObstacleAvoidance.run()

if __name__ == '__main__':
	main()
