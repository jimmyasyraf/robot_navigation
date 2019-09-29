#!/usr/bin/python
from ultrasonic import Ultrasonic
from motor import Motor
import RPi.GPIO as GPIO
import time
import rospy
from robot_navigation.msg import Distance
from robot_navigation.msg import Direction

# Center Ultrasonic
TRIG1 = 4
ECHO1 = 17

# Right Ultrasonic
TRIG2 = 27
ECHO2 = 22

# Left Ultrasonic
TRIG3 = 19
ECHO3 = 26

# Motor 1
MOTOR1A = 18
MOTOR1B = 23

# Motor 2
MOTOR2A = 16
MOTOR2B = 20

# Motor matrix
FORWARD = [False, True, False, True]
BACKWARD = [True, False, True, False]
RIGHT = [False, True, True, False]
LEFT = [True, False, False, True]
STOP = [False, False, False, False]

# Directions
FORWARD_DIRECTION = 1
RIGHT_DIRECTION = 2
LEFT_DIRECTION = 3
BACKWARD_DIRECTION = 4
STOP_DIRECTION = 5

def exit_gracefully():
	GPIO.cleanup()

def scan_distance():
	distance1 = ultrasonic1.get_range()
	distance2 = ultrasonic2.get_range()
	distance3 = ultrasonic3.get_range()
	return distance1, distance2, distance3;

def steer(data):
	direction = data.direction
	if direction == FORWARD_DIRECTION:
		motors.move(FORWARD)
	elif direction == RIGHT_DIRECTION:
		motors.move(RIGHT)
	elif direction == LEFT_DIRECTION:
		motors.move(LEFT)
	elif direction == BACKWARD_DIRECTION:
		motors.move(BACKWARD)
	elif direction == STOP_DIRECTION:
		motors.move(STOP)

def run():
	global ultrasonic1, ultrasonic2, ultrasonic3, motors
	ultrasonic1 = Ultrasonic(TRIG1, ECHO1)
	ultrasonic2 = Ultrasonic(TRIG2, ECHO2)
	ultrasonic3 = Ultrasonic(TRIG3, ECHO3)
	motors = Motor(MOTOR1A, MOTOR1B, MOTOR2A, MOTOR2B)

	rospy.init_node('controller', anonymous=True)
	distance_publisher = rospy.Publisher('distance', Distance, queue_size=10)
	rospy.Subscriber('direction', Direction, steer)
	rospy.on_shutdown(exit_gracefully)
	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		distance1, distance2, distance3 = scan_distance()
		distance_msg = Distance()
		distance_msg.distance1 = distance1
		distance_msg.distance2 = distance2
		distance_msg.distance3 = distance3
		rospy.loginfo(distance_msg)
		distance_publisher.publish(distance_msg)
		rate.sleep()

if __name__ == '__main__':
	run()
