#!/usr/bin/python
#Object for commanding the robot
#Framework by: Morgan Dykshorn

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
from time import sleep
import math

class KhanMove:
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

	def forward(self, motorspeed):
		
		#pwm for motor 1 and 2
		PWM.start("P9_14", motorspeed)
		PWM.start("P9_16", motorspeed)
		#pwm for motor 3 and 4
		PWM.start("P8_13", motorspeed)
		PWM.start("P8_19", motorspeed)

		#left Side
		#writes to motor 1
		GPIO.output("P9_11", 0)
		GPIO.output("P9_12", 1)
		#writes to motor 2
		GPIO.output("P9_13", 0)
		GPIO.output("P9_15", 1)
		#right side
		#writes to motor 3
		GPIO.output("P8_14", 1)
		GPIO.output("P8_16", 0)
		#writes to motor 4
		GPIO.output("P8_10", 1)
		GPIO.output("P8_18", 0)
	def reverse(self, motorspeed):
		
		#pwm for motor 1 and 2
		PWM.start("P9_14", motorspeed)
		PWM.start("P9_16", motorspeed)
		#pwm for motor 3 and 4
		PWM.start("P8_13", motorspeed)
		PWM.start("P8_19", motorspeed)


		#left Side
		#writes to motor 1
		GPIO.output("P9_11", 1)
		GPIO.output("P9_12", 0)
		#writes to motor 2
		GPIO.output("P9_13", 1)
		GPIO.output("P9_15", 0)
		#right side
		#writes to motor 3
		GPIO.output("P8_14", 0)
		GPIO.output("P8_16", 1)
		#writes to motor 4
		GPIO.output("P8_10", 0)
		GPIO.output("P8_18", 1)


	def right(self, motorspeed):
		
		#pwm for motor 1 and 2
		PWM.start("P9_14", motorspeed)
		PWM.start("P9_16", motorspeed)
		#pwm for motor 3 and 4
		PWM.start("P8_13", motorspeed)
		PWM.start("P8_19", motorspeed)

		#left Side
		#writes to motor 1
		GPIO.output("P9_11", 0)
		GPIO.output("P9_12", 1)
		#writes to motor 2
		GPIO.output("P9_13", 0)
		GPIO.output("P9_15", 1)
		#right side
		#writes to motor 3
		GPIO.output("P8_14", 0)
		GPIO.output("P8_16", 1)
		#writes to motor 4
		GPIO.output("P8_10", 0)
		GPIO.output("P8_18", 1)


	def left(self, motorspeed):
		
		#pwm for motor 1 and 2
		PWM.start("P9_14", motorspeed)
		PWM.start("P9_16", motorspeed)
		#pwm for motor 3 and 4
		PWM.start("P8_13", motorspeed)
		PWM.start("P8_19", motorspeed)

		#left Side
		#writes to motor 1
		GPIO.output("P9_11", 1)
		GPIO.output("P9_12", 0)
		#writes to motor 2
		GPIO.output("P9_13", 1)
		GPIO.output("P9_15", 0)
		#right side
		#writes to motor 3
		GPIO.output("P8_14", 1)
		GPIO.output("P8_16", 0)
		#writes to motor 4
		GPIO.output("P8_10", 1)
		GPIO.output("P8_18", 0)


	def stop(self):

		#left Side
		#writes to motor 1
		GPIO.output("P9_11", 0)
		GPIO.output("P9_12", 0)
		#writes to motor 2
		GPIO.output("P9_13", 0)
		GPIO.output("P9_15", 0)
		#right side
		#writes to motor 3
		GPIO.output("P8_14", 0)
		GPIO.output("P8_16", 0)
		#writes to motor 4
		GPIO.output("P8_10", 0)
		GPIO.output("P8_18", 0)

		#pwm for motor 1 and 2
		PWM.stop("P9_14")
		PWM.stop("P9_16")
		#pwm for motor 3 and 4
		PWM.stop("P8_13")
		PWM.stop("P8_19")

class IRdistance:
	def __init__(self):
		ADC.setup()

	def readdist(self):
		#reads voltage value
		voltage = ADC.read("P9_40")

		#converts voltage values into distance(meters)
		distance = (voltage**-.8271732796)
		distance = distance*.1679936709

		return distance

