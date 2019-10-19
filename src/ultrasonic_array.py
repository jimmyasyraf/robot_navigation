#!/usr/bin/python

from ultrasonic import Ultrasonic
import math
import rospy
from sensor_msgs.msg import Range

NUM_ULTRASONIC = 3

TRIGGER_CENTER = 4
ECHO_CENTER = 17

TRIGGER_RIGHT = 27
ECHO_RIGHT = 22

TRIGGER_LEFT = 19
ECHO_LEFT = 26

class UltrasonicArray():
	def __init__(self, num_ultrasonic, trigger_list, echo_list, range_min, range_max, angle_min, angle_max):
		self.ultrasonic_array = []
		self.publisher_array = []
		self.num_ultrasonic = num_ultrasonic

		delta_angle = (angle_max-angle_min)/float(num_ultrasonic-1)

		rospy.loginfo("Initializing the arrays")

		for i in range(num_ultrasonic):
			ultrasonic = Ultrasonic(trigger_list[i], echo_list[i], range_min, range_max)
			angle = angle_min + delta_angle*i
			ultrasonic.angle = math.radians(angle)
			self.ultrasonic_array.append(ultrasonic)
			rospy.loginfo("Sonar %d set"%i)

			topic_name = "/ultrasonic/%d"%i
			publisher = rospy.Publisher(topic_name, Range, queue_size=10)
			self.publisher_array.append(publisher)
			rospy.loginfo("Publisher %d set with topic %s"%(i, topic_name))

		message = Range()
		message.radiation_type = 0
		message.min_range = range_min
		message.max_range = range_max
		self._message = message

	def scan(self):
		range_array = []

		for i in range(self.num_ultrasonic):
			range_cm = self.ultrasonic_array[i].get_range()
			range_array.append(range_cm)
			self._message.range = range_cm
			self._message.field_of_view = self.ultrasonic_array[i].angle
			self.publisher_array[i].publish(self._message)

		rospy.loginfo("Range (cm): left = %4.2f center = %4.2f right = %4.2f"%(range_array[0], range_array[1], range_array[2]))

	def run(self):
		rate = rospy.Rate(10)
		rospy.loginfo("Running...")


		while not rospy.is_shutdown():
			self.scan()
			rate.sleep()

		rospy.loginfo("Stopped")

if __name__ == "__main__":
	rospy.loginfo("Setting up ultrasonic node")
	rospy.init_node('ultrasonic_array')

	trigger_list = [TRIGGER_LEFT, TRIGGER_CENTER, TRIGGER_RIGHT]
	echo_list = [ECHO_LEFT, ECHO_CENTER, ECHO_RIGHT]

	ultrasonic_array = UltrasonicArray(NUM_ULTRASONIC, trigger_list, echo_list, 5, 400, -30, 30)
	ultrasonic_array.run()
