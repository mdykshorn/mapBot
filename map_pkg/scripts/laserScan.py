#!/usr/bin/python
#stepper framework by petebachant
#scanning and changes made by Morgan Dykshorn
#imports necessary libraries
import time
import math
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO

def initialize_pins(pins):
	for pin in pins:
		GPIO.setup(pin, GPIO.OUT)

def set_all_pins_low(pins):
	for pin in pins:
		GPIO.output(pin, GPIO.LOW)
		
def wavedrive(pins, pin_index):
	for i in range(len(pins)):
		if i == pin_index:
			GPIO.output(pins[i], GPIO.HIGH)
		else:
			GPIO.output(pins[i], GPIO.LOW)

def fullstep(pins, pin_index):
	"""pin_index is the lead pin"""
	GPIO.output(pins[pin_index], GPIO.HIGH)
	GPIO.output(pins[(pin_index+3) % 4], GPIO.HIGH)
	GPIO.output(pins[(pin_index+1) % 4], GPIO.LOW)
	GPIO.output(pins[(pin_index+2) % 4], GPIO.LOW)


class laserScan(object):
	def __init__(self, steps_per_rev=2048.0,
				 pins=["P9_27", "P8_15", "P8_11", "P8_12"]):

		self.pins = pins
		
		initialize_pins(self.pins)
		set_all_pins_low(self.pins)
		
		self.angle = 0
		self.steps_per_rev = steps_per_rev
		
		# Initialize stepping mode
		self.drivemode = fullstep

		#sets up ADC
		ADC.setup()
		#cretes a value for the threshold
		self.threshold = .2
		#creates a variable for the distance measurment
		self.distance = 0

	def calibrate(self):
		#sets angle to 1 so loop will run
		self.angle = 1.0

		while self.angle!=0:

			for pin_index in range(len(self.pins)):
				self.drivemode(self.pins, pin_index)
				#waits for .0025 seconds keeping the sensor reading at 100hz
				time.sleep(.0025)
				#checks for when the sensor triggers 0 angle calibration
				#reads value from sensor
				rotateVal = ADC.read("P9_39")
				if rotateVal < self.threshold:
					self.angle = 0			
		set_all_pins_low(self.pins)	

	
	def scan(self):
		
		#runs indefinitely
		while 1:
			#set step to 0
			step = 0
			# sets ammount of steps for each time function is called
			steps = 4
			self.direction = 1
		
			while step < steps:
				for pin_index in range(len(self.pins)):
					self.drivemode(self.pins, pin_index)
					#waits for .0025 seconds keeping the sensor reading at 100hz
					time.sleep(.0025)
					step += 1
					#checks for when the sensor triggers 0 angle calibration
					#reads value from sensor
					rotateVal = ADC.read("P9_39")
					if rotateVal < self.threshold:
						self.angle = 0
						step = 0
					else:
						self.angle = self.angle + .9


			#read data from LIDAR (need to figure out how)
				
			#set the read distance to the distance variable
			self.distance = 2.0

			print self.angle, self.distance
	
		set_all_pins_low(self.pins)	


if __name__ == '__main__':

	#creates an instance of the class
	lScan = laserScan()

	lScan.calibrate()

	lScan.scan()