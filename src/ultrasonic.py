#!/usr/bin/python
import RPi.GPIO as GPIO
import time

class Ultrasonic():
	def __init__(self, trigger, echo, range_min=0, range_max=400):
		GPIO.setmode(GPIO.BCM)

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

	@property
	def is_reading(self):
		return(self.is_reading)

