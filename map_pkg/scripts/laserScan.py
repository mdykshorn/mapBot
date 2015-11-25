#!/usr/bin/python
#imports necessary libraries
import time
from bbpystepper import Stepper
import Adafruit_BBIO.ADC as ADC


class laserScan:
	def __init__(self):
		#sets up ADC
		ADC.setup()
		#creates an instance of the stepper class
		mystepper = Stepper()
		#cretes a value for the threshold
		self.threshold = .7
		#creates a variable for the current degree measurement
		self._degree = none
		self._distance = none


	def calibrate(self):

		while mystepper.angle~=0:

			#rotates the steper 3.6 degrees or 4 steps(after gearing)
			mystepper.rotate(3.6, 150)
			#reads value from sensor
			rotateVal = ADC.read("P9_39")
			#checks if value is below the threshold
			if rotateVal < self.threshold:
				mystepper.zero_angle()
			#sleeps for .01 seconds to not overload motor
			time.sleep(.01)


	#function that scans with inputs of speed(in rpm)
	def scan(self, speed):
		#reads value from sensor
		rotateVal = ADC.read("P9_39")
		#reads value from sensor
		if rotateVal < self.threshold:
			mystepper.zero_angle()

		#read data from LIDAR (need to figure out how)

		#set the degree measurment to the current calculated angle
		self._degree = mystepper.angle
		self._distance = 2 #temporarily assigns distance to 2m

		#rotates the steper 3.6 degrees or 4 steps(after gearing)
		mystepper.rotate(3.6, speed)

	@property
	def degree(self):
		return self._degree
	@property
	def distance(self):
		return self._distance	


if __name__ == '__main__':

	creates an instance of the class
	lScan = laserscan()

	lScan.calibrate()

	scantime = input('how many data points')
	count = 0

	#runs while count is less than the number of datapoints
	while count<scantime:
		lScan.scan(150)
		Distance = lScan.distance()
		Degree = lScan.degree()

		print Degree, Distance

		#sleeps for .01 seconds, keeping the rate of scanning 100hz
		time.sleep(.5)



