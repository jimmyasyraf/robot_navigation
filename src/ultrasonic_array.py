#!/usr/bin/python

import math
import time
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range

NUM_ULTRASONIC = 3

TRIGGER_CENTER = 4
ECHO_CENTER = 17

TRIGGER_RIGHT = 27
ECHO_RIGHT = 22

TRIGGER_LEFT = 19
ECHO_LEFT = 26

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def exit_gracefully():
	GPIO.cleanup()

class Ultrasonic():
	def __init__(self, trigger, echo, range_min=0, range_max=400):
		self._trigger = trigger
		self._echo = echo
		self._range_min = range_min
		self._range_max = range_max
		self._is_reading = False
		self._speed_sound = 17150
		self._last_time_reading = 0
		self._timeout = range_max/self._speed_sound*2

		GPIO.setup(trigger, GPIO.OUT)
		GPIO.setup(echo, GPIO.IN)

		GPIO.output(trigger, GPIO.LOW)
		time.sleep(1)

	def get_range(self):
		self._is_reading = True

		GPIO.output(self._trigger, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(self._trigger, GPIO.LOW)

		GPIO.output(self._trigger, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(self._trigger, GPIO.LOW)

		pulse_start_time = time.time()
		pulse_end_time = time.time()

		while GPIO.input(self._echo)==0:
			pulse_start_time = time.time()

		while GPIO.input(self._echo)==1:
			pulse_end_time = time.time()

		self._last_time_reading = time.time()
		self._is_reading = False

		pulse_duration = pulse_end_time - pulse_start_time
		distance = pulse_duration * self._speed_sound

		if distance > self._range_max:
			distance = self._range_max

		if distance < self._range_min:
			distance = self._range_min

		return(distance)

class UltrasonicArray():
	def __init__(self, num_ultrasonic, trigger_list, echo_list, range_min, range_max, angle_min, angle_max):
		self.ultrasonic_array = []
		self.publisher_array = []
		self.num_ultrasonic = num_ultrasonic

		delta_angle = (angle_max-angle_min)/float(num_ultrasonic-1)

		rospy.loginfo("Initializing the arrays")

		for i in range(num_ultrasonic):
			ultrasonic = Ultrasonic(trigger_list[i], echo_list[i], range_min*100, range_max*100)
			angle_deg = angle_min + delta_angle*i
			ultrasonic.angle = math.radians(angle_deg)
			self.ultrasonic_array.append(ultrasonic)
			rospy.loginfo("Sonar %d set"%i)

			topic_name = "/robot/ultrasonic/%d"%i
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
			range_array.append(range_cm*0.01)
			self._message.range = range_cm*0.01
			self._message.field_of_view = self.ultrasonic_array[i].angle
			self.publisher_array[i].publish(self._message)

		rospy.loginfo("Range (m): left = %4.2f center = %4.2f right = %4.2f"%(range_array[0], range_array[1], range_array[2]))

	def run(self):
		rate = rospy.Rate(10)
		rospy.on_shutdown(exit_gracefully)
		rospy.loginfo("Running...")


		while not rospy.is_shutdown():
			self.scan()
			rate.sleep()

		rospy.loginfo("Stopped")

def main():
	rospy.loginfo("Setting up ultrasonic node")
	rospy.init_node('ultrasonic_array')

	trigger_list = [TRIGGER_LEFT, TRIGGER_CENTER, TRIGGER_RIGHT]
	echo_list = [ECHO_LEFT, ECHO_CENTER, ECHO_RIGHT]

	ultrasonic_array = UltrasonicArray(NUM_ULTRASONIC, trigger_list, echo_list, 0.05, 3.0, -30, 30)
	ultrasonic_array.run()

if __name__ == '__main__':
	main()
