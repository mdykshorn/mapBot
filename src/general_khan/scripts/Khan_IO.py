#!/usr/bin/python
#Object for motor driving on Khan
#Framework by: Morgan Dykshorn

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM

#class that lets you individually control the left and right side motors
class motorDrive:
	def __init__(self):

		#sets up GPIO pins for motors 1 and 2
		GPIO.setup("P9_11", GPIO.OUT)
		GPIO.setup("P9_12", GPIO.OUT)
		GPIO.setup("P9_13", GPIO.OUT)
		GPIO.setup("P9_15", GPIO.OUT)

		#sets up GPIO pins for motors 3 and 4
		GPIO.setup("P8_14", GPIO.OUT)
		GPIO.setup("P8_16", GPIO.OUT)
		GPIO.setup("P8_10", GPIO.OUT)
		GPIO.setup("P8_18", GPIO.OUT)
		
		#initilizes PWM
		PWM.start("P9_14", 0)
		PWM.start("P9_16", 0)
		PWM.start("P8_13", 0)		
		PWM.start("P8_19", 0)
		
	def leftFront(self, motorspeed):
	#motorspeed -100-100,

		if motorspeed < 0:
				Channel_A = 1
				Channel_B = 0
		elif motorspeed > 0:
				Channel_A = 0
				Channel_B = 1
		elif motorspeed == 0:
			Channel_A = 0
			Channel_B = 0
		#convert speed to a positive number
		motorspeed = abs(motorspeed)
		
		#makes sure motorspeed doesn't go out of range
		if motorspeed > 100:
			motorspeed = 100
		
		#pwm for motor 1
		PWM.set_duty_cycle("P9_14", motorspeed)

		#left Side
		#writes to motor 1
		GPIO.output("P9_11", Channel_A)
		GPIO.output("P9_12", Channel_B)

		
	def leftRear(self, motorspeed):
	#motorspeed -100-100,

		if motorspeed < 0:
				Channel_A = 1
				Channel_B = 0
		elif motorspeed > 0:
				Channel_A = 0
				Channel_B = 1
		elif motorspeed == 0:
			Channel_A = 0
			Channel_B = 0

		#convert speed to a positive number
		motorspeed = abs(motorspeed)

		#makes sure motorspeed doesn't go out of range
		if motorspeed > 100:
			motorspeed = 100
		
		#pwm for motor 2
		PWM.set_duty_cycle("P9_16", motorspeed)

		#left Side
		#writes to motor 2
		GPIO.output("P9_13", Channel_A)
		GPIO.output("P9_15", Channel_B)	

	def rightFront(self, motorspeed):
	#motorspeed -100-100,

		if motorspeed < 0:
				Channel_A = 0
				Channel_B = 1
		elif motorspeed > 0:
				Channel_A = 1
				Channel_B = 0
		elif motorspeed == 0:
			Channel_A = 0
			Channel_B = 0

		#convert speed to a positive number
		motorspeed = abs(motorspeed)
		
		#makes sure motorspeed doesn't go out of range
		if motorspeed > 100:
			motorspeed = 100
		
		#pwm for motor 3
		PWM.set_duty_cycle("P8_13", motorspeed)

		#right side
		#writes to motor 3
		GPIO.output("P8_14", Channel_A)
		GPIO.output("P8_16", Channel_B)


	def rightRear(self, motorspeed):
	#motorspeed -100-100,

		if motorspeed < 0:
				Channel_A = 0
				Channel_B = 1
		elif motorspeed > 0:
				Channel_A = 1
				Channel_B = 0
		elif motorspeed == 0:
			Channel_A = 0
			Channel_B = 0

		#convert speed to a positive number
		motorspeed = abs(motorspeed)
		
		#makes sure motorspeed doesn't go out of range
		if motorspeed > 100:
			motorspeed = 100
		
		#pwm for motor 4
		PWM.set_duty_cycle("P8_19", motorspeed)

		#right side
		#writes to motor 4
		GPIO.output("P8_10", Channel_A)
		GPIO.output("P8_18", Channel_B)
