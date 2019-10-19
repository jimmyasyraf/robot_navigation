#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

MOTOR_RIGHT_IN1 = 18
MOTOR_RIGHT_IN2 = 23
MOTOR_RIGHT_EN = 24

MOTOR_LEFT_IN1 = 16
MOTOR_LEFT_IN2 = 20
MOTOR_LEFT_EN = 21

FREQUENCY = 1000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def exit_gracefully():
	GPIO.cleanup()

def clip(value, minimum, maximum):
	if value < minimum:
		return minimum
	elif value > maximum:
		return maximum
	return value

class Motor:
	def __init__(self, in1, in2, en):
		self._in1 = in1
		self._in2 = in2
		self._en = en

		GPIO.setup(in1, GPIO.OUT)
		GPIO.setup(in2, GPIO.OUT)
		GPIO.setup(en, GPIO.OUT)

		self._pwm = GPIO.PWM(en, FREQUENCY)

	def move(self, speed_percent):
		speed = clip(abs(speed_percent), 0, 100)

		if speed_percent < 0:
			# Backward
			GPIO.output(self._in1, GPIO.LOW)
			GPIO.output(self._in2, GPIO.HIGH)
			self._pwm.start(speed)
		else:
			# Forward
			GPIO.output(self._in1, GPIO.HIGH)
			GPIO.output(self._in2, GPIO.LOW)
			self._pwm.start(speed)

class Driver:
	def __init__(self):
		rospy.init_node('driver')
		self._last_received = rospy.get_time()
		self._timeout = rospy.get_param('~timeout', 2)
		self._rate = rospy.get_param('~rate', 10)
		self._max_speed = rospy.get_param('~max_speed', 0.5)
		self._wheel_base = rospy.get_param('~wheel_base', 0.26)

		self._left_motor = Motor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN)
		self._right_motor = Motor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN)

		self._left_speed_percent = 0
		self._right_speed_percent = 0

		rospy.Subscriber('/robot/cmd_vel', Twist, self.velocity_received_callback)

	def velocity_received_callback(self, message):
		self._last_received = rospy.get_time()

		linear = message.linear.x
		angular = message.angular.z

		left_speed = linear - angular*self._wheel_base
		right_speed = linear + angular*self._wheel_base

		self._left_speed_percent = (100 * left_speed/self._max_speed)
		self._right_speed_percent = (100 * right_speed/self._max_speed)

	def run(self):
		rate = rospy.Rate(self._rate)
		rospy.on_shutdown(exit_gracefully)

		while not rospy.is_shutdown():
			delay = rospy.get_time() - self._last_received
			if delay < self._timeout:
				self._left_motor.move(self._left_speed_percent)
				self._right_motor.move(self._right_speed_percent)
			else:
				self._left_motor.move(0)
				self._right_motor.move(0)
			rate.sleep()

def main():
	driver = Driver()
	driver.run()

if __name__ == '__main__':
	main()
