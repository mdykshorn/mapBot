#!/usr/bin/python
#stepper framework by petebachant
#scanning and changes made by Morgan Dykshorn
#imports necessary libraries
import time
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_I2C import Adafruit_I2C


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
	def __init__(self,
				 pins=["P9_27", "P8_15", "P8_11", "P8_12"]):

		#sets lidar lite addresses
		self.address = 0x62
		self.distWriteReg = 0x00
		self.distWriteVal = 0x04
		self.distReadReg1 = 0x8f
		self.distReadReg2 = 0x10

		#initilizes the i2c bus on address lidar lite address on bus 1
		#self.i2c = Adafruit_I2C(self.address, 1)
		
		#creates pins for the stepper motors
		self.pins = pins
		
		#initlizes the pins for stepper
		initialize_pins(self.pins)
		set_all_pins_low(self.pins)
		
		#sets angle to 0
		self.angle = 0
		
		# Initialize stepping mode
		self.drivemode = fullstep

		#sets up ADC
		ADC.setup()
		#cretes a value for the threshold
		self.threshold = .7
		#creates a variable for the distance measurment
		self.distance = 0
		#creates a variable for the last ir sensor read
		self.lastval = 1

		#creates a variable for the ROS message and initilizes constant parameters
		self.msg = LaserScan()
		self.header = Header()

		self.msg.angle_increment = 0.015708
		self.msg.range_min = 0
		self.msg.range_max = 10


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
				rotateVal = ADC.read("P9_37")
				if rotateVal > self.threshold and self.lastval<self.threshold:
					self.angle = 0
				self.lastval = rotateVal			
		set_all_pins_low(self.pins)	

	
	def scan(self):
		#initilizes a count for number of datapoints
		count = 0
		#changes angle slightly so loop will run
		self.angle = .0001
		#creates a variable that returns 1 when the scan completes
		scanComplete = 0
		#runs while there are less than 400 data points or the angle becomes is not 0
		while count<400 and self.angle != 0:
	
			for pin_index in range(len(self.pins)):
				self.drivemode(self.pins, pin_index)
				
				#writes to the LIDAR to take a reading
				#writes to the register that takes measurment
				#self.i2c.write8(self.distWriteReg, self.distWriteVal)
				#waits for 'x' seconds to control motor speed and prevent sensor overpolling(minimum sleep time is .0025)
				time.sleep(.0025)
				
				#checks for when the sensor triggers 0 angle calibration
				#reads value from sensor
				rotateVal = ADC.read("P9_37")
				#checks if the sensor is over the threshold but the last value is under the threshold
				#if both are true the sensor has just been passe dan should now reset
				if rotateVal > self.threshold and self.lastval<self.threshold:
					self.angle = 0
				else:
					self.angle = self.angle + .9
				self.lastval = rotateVal

				
				#reads from ir sensor
				#reads voltage value
				voltage = ADC.read("P9_39")
				#converts voltage values into distance(meters)
				self.distance = (voltage*17.546)
				#checks and discards data outside of accurate range
				if self.distance>10:
					self.distance = 10
				elif self.distance<.1:
					self.distance = .1
					
				#writes dynamic data to msg
				#uses final data gathering time for the time increment between data readings
				#taking the average over the entire period would probably be more accurate
				self.msg.ranges.append(self.distance)
				
				count = count+1

		#sets pin low when scan completes
		set_all_pins_low(self.pins)

		#changes variable when scan is complete
		scanComplete = 1
		return scanComplete	


if __name__ == '__main__':
	try:
		#initializes the node named scanner
		rospy.init_node('irdistance', anonymous=True)

		pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

		#creates an instance of the class
		lScan = laserScan()

		lScan.calibrate()

		scanComplete = 0
		startTime = 0
		endTIme = 0
		scanTime = 0

		#create sequence for message
		sequence = 0

		#keeps loop running
		while not rospy.is_shutdown():

			startTime = rospy.get_time()
			scanComplete = lScan.scan()
			endTime = rospy.get_time()

			scanTime = endTime - startTime

			#creates message header
			lScan.msg.header.seq = sequence
			lScan.msg.header.stamp.secs = lScan.startTime
			lScan.msg.header.frame_id = "/base_link"
			#hardcodes angle values, might differ by +- 1 degree but that is insignificant
			lscan.msg.angle_min = .0157077
			lscan.msg.angle_max = 6.28308
			#calculates time increment
			lscan.msg.time_increment = scanTime / 400
			lscan.msg.scan_time - scanTime



			pub.publish(lScan.msg)


			sequence = sequence+1


		#keeps the node from quitting
		rospy.spin()
	except rospy.ROSInterruptException:
	    pass