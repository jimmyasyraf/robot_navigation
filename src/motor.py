#!/usr/bin/python
import RPi.GPIO as GPIO
import time

class Motor():
	def __init__(self, gpio_motor1a, gpio_motor1b, gpio_motor2a, gpio_motor2b):
		GPIO.setmode(GPIO.BCM)
		self._gpio_motor1a = gpio_motor1a
		self._gpio_motor1b = gpio_motor1b
		self._gpio_motor2a = gpio_motor2a
		self._gpio_motor2b = gpio_motor2b

		GPIO.setup(gpio_motor1a, GPIO.OUT)
		GPIO.setup(gpio_motor1b, GPIO.OUT)
		GPIO.setup(gpio_motor2a, GPIO.OUT)
		GPIO.setup(gpio_motor2b, GPIO.OUT)

	def move(self, direction):
		GPIO.output(self._gpio_motor1a, direction[0])
		GPIO.output(self._gpio_motor1b, direction[1])
		GPIO.output(self._gpio_motor2a, direction[2])
		GPIO.output(self._gpio_motor2b, direction[3])

