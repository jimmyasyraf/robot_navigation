#!/usr/bin/python
import RPi.GPIO as GPIO          
from time import sleep

# Motor 1
MOTOR1A = 18
MOTOR1B = 23
EN1 = 24
temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR1A, GPIO.OUT)
GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(EN1, GPIO.OUT)
GPIO.output(MOTOR1A, GPIO.LOW)
GPIO.output(MOTOR1B, GPIO.LOW)
p=GPIO.PWM(EN1,1000)
p.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")

while(1):
  x=raw_input()

  if x=='r':
    print("run")
    if(temp1==1):
      GPIO.output(MOTOR1A,GPIO.HIGH)
      GPIO.output(MOTOR1B,GPIO.LOW)
      print("forward")
      x='z'
    else:
      GPIO.output(MOTOR1A,GPIO.LOW)
      GPIO.output(MOTOR1B,GPIO.HIGH)
      print("backward")
      x='z'

  elif x=='s':
    print("stop")
    GPIO.output(MOTOR1A,GPIO.LOW)
    GPIO.output(MOTOR1B,GPIO.LOW)
    x='z'
  
  elif x=='f':
    print("forward")
    GPIO.output(MOTOR1A,GPIO.HIGH)
    GPIO.output(MOTOR1B,GPIO.LOW)
    temp1=1
    x='z'

  elif x=='b':
    print("backward")
    GPIO.output(MOTOR1A,GPIO.LOW)
    GPIO.output(MOTOR1B,GPIO.HIGH)
    temp1=0
    x='z'

  elif x=='l':
    print("low")
    p.ChangeDutyCycle(25)
    x='z'

  elif x=='m':
    print("medium")
    p.ChangeDutyCycle(50)
    x='z'

  elif x=='h':
    print("high")
    p.ChangeDutyCycle(75)
    x='z'

  elif x=='e':
    GPIO.cleanup()
    break

  else:
    print("<<<  wrong data  >>>")
    print("please enter the defined data to continue.....")
