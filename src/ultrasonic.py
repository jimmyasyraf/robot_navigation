#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import rospy
from robot_navigation.msg import Distance
from robot_navigation.msg import Direction

# center
TRIG1 = 4
ECHO1 = 17

# right
TRIG2 = 27
ECHO2 = 22

# left
TRIG3 = 19
ECHO3 = 26

class Ultrasonic():
	def __init__(self, gpio_trigger, gpio_echo, range_min=0, range_max=400):
		GPIO.setmode(GPIO.BCM)

		self._gpio_trigger = gpio_trigger
		self._gpio_echo = gpio_echo
		self._range_min = range_min
		self._range_max = range_max
		self._is_reading = False
		self._speed_sound = 17150
		self._last_time_reading = 0
		self._timeout = range_max/self._speed_sound*2

		GPIO.setup(gpio_trigger, GPIO.OUT)
		GPIO.setup(gpio_echo, GPIO.IN)

		GPIO.output(gpio_trigger, GPIO.LOW)
		time.sleep(1)

	def get_range(self):
		self._is_reading = True

		GPIO.output(self._gpio_trigger, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(self._gpio_trigger, GPIO.LOW)

		GPIO.output(self._gpio_trigger, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(self._gpio_trigger, GPIO.LOW)

		pulse_start_time = time.time()
		pulse_end_time = time.time()

		while GPIO.input(self._gpio_echo)==0:
			pulse_start_time = time.time()

		while GPIO.input(self._gpio_echo)==1:
			pulse_end_time = time.time()

		self._last_time_reading = time.time()
		self._is_reading = False

		pulse_duration = pulse_end_time - pulse_start_time
		distance = pulse_duration * self._speed_sound

		if distance > self._range_max:
			distance = self._range_max

		if distance < self._range_min:
			distance = self._range_min

		distance = int(round(distance))
		return(distance)

	@property
	def is_reading(self):
		return(self.is_reading)

def exit_gracefully():
	GPIO.cleanup()

def move():
	distance_publisher = rospy.Publisher('distance', Distance, queue_size=10)
	rospy.init_node('controller', anonymous=True)
	rospy.on_shutdown(exit_gracefully)
	rate = rospy.Rate(5)

	ultrasonic1 = Ultrasonic(TRIG1, ECHO1)
	ultrasonic2 = Ultrasonic(TRIG2, ECHO2)
	ultrasonic3 = Ultrasonic(TRIG3, ECHO3)

	while not rospy.is_shutdown():
		distance1 = ultrasonic1.get_range()
		distance2 = ultrasonic2.get_range()
		distance3 = ultrasonic3.get_range()
		distance_msg = Distance()
		distance_msg.distance1 = distance1
		distance_msg.distance2 = distance2
		distance_msg.distance3 = distance3
		rospy.loginfo(distance_msg)
		distance_publisher.publish(distance_msg)
		rate.sleep()

if __name__ == '__main__':
	move()
