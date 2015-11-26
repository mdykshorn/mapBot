#!/usr/bin/python
#stepper framework by petebachant
#scanning and changes made by Morgan Dykshorn
#imports necessary libraries
import time
import math
#import rospy
#from sensor_msgs.msg import LaserScan
#from std_msgs.msg import Header
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_I2C import Adafruit_I2C

#ALL ROS commands are temprorarily commented out for testing purposes

# Create variable so we can always see/use it, but set it to a value that indicates it's not yet valid
pub = None

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

#temporarily defines the laserscan message
class message(object):
	def __init__(self):
		self.angle_min = 0 			#can delete later
		self.angle_max = 0 			#can delete later
		self.angle_increment = 0.015708
		self.time_increment = 0 	#can delete later
		self.scan_time = 0 			#can delte later
		self.range_min = 0
		self.range_max = 40
		self.ranges = []

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
		self.i2c = Adafruit_I2C(self.address, 1)
		
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
		self.threshold = .2
		#creates a variable for the distance measurment
		self.distance = 0
		#creates a variable for the last ir sensor read
		self.lastval = 1
		#creates variables for time
		self.startTime = 0.0
		self.finishTime = 0.0
		self.datatime1 = 0.0
		self.datatime2 = 0.0
		#creates a list of all angles throughout the scan (saves angle in radians)
		self.angleR = []

		#creates a variable for the ROS message and initilizes constant parameters
		#self.msg = LaserScan()
		#self.header = Header()
		self.msg = message()				#can delete later
		self.msg.angle_increment = 0.015708
		self.msg.range_min = 0
		self.msg.range_max = 40


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
				if rotateVal > self.threshold and self.lastval<self.threshold:
					self.angle = 0
				self.lastval = rotateVal			
		set_all_pins_low(self.pins)	

	
	def scan(self):
		#initilizes a count for number of datapoints
		count = 0
		#changes angle temporarily so loop will run
		self.angle = .01
		#creates a variable that returns 1 when the scan completes
		scanComplete = 0
		#runs indefinitely
		#records the start time 
		#self.startTime = rospy.get_time()
		self.startTime = time.clock()		#can delete later
		while count<400 or self.angle != 0:
	
			for pin_index in range(len(self.pins)):
				#gets time before first reading
				#self.datatime1 = rospy.get_time()
				self.datatime1 = time.clock()		#can delete later
				self.drivemode(self.pins, pin_index)
				
				#writes to the LIDAR to take a reading
				#writes to the register that takes measurment
				self.i2c.write8(self.distWriteReg, self.distWriteVal)
				#waits for 'x' seconds to control motor speed and prevent sensor overpolling(minimum sleep time is .0025)
				time.sleep(.01)
				
				#checks for when the sensor triggers 0 angle calibration
				#reads value from sensor
				rotateVal = ADC.read("P9_39")
				#checks if the sensor is over the threshold but the last value is under the threshold
				#if both are true the sensor has just been passe dan should now reset
				if rotateVal > self.threshold and self.lastval<self.threshold:
					self.angle = 0
				else:
					self.angle = self.angle + .9
				self.lastval = rotateVal
				#calculates the angle in radians and saves it to a list
				self.angleR.append(self.angle*0.0174533)
				#reads distance from Lidar
				dist1 = self.i2c.readU8(self.distReadReg1)
				dist2 = self.i2c.readU8(self.distReadReg2)
				#shifts bits to get correct distance
				self.distance = (dist1<<8) + dist2

	
				#Writes dynamic data to laserscan message
				self.msg.ranges.append(self.distance)
				#saves previous start time
				self.datatime2 = self.datatime1
				count = count+1

		#sets pin low when scan completes
		set_all_pins_low(self.pins)

		#writes static data to laserscan message
		self.msg.angle_min = self.angleR[0]
		self.msg.angle_max = self.angleR[count-1]
		#uses final data gathering time for the time increment between data readings
		#taking the average over the entire period would probably be more accurate
		self.msg.time_increment = (self.datatime1 - self.datatime2)

		#records final time
		#self.finishTime = rospy.get_time()
		self.finishTime = time.clock()	#can delete later
		#calcualtes scan time
		self.msg.scan_time = (self.finishTime - self.startTime)

		#changes variable when scan is complete
		scanComplete = 1
		return scanComplete	


if __name__ == '__main__':

	#ALL ROS commands are temprorarily commented out for testing purposes

	#initializes the node named scanner
	#rospy.init_node('irdistance', anonymous=True)
	# Declare we are using the pub defined above, not a new local variable
	#global pub
	#pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

	#creates an instance of the class
	lScan = laserScan()

	lScan.calibrate()

	scanComplete = 0

	#create sequence for message
	sequence = 0

	#keeps loop running
	#while not rospy.is_shutdown():
	while 1:
		scanComplete = lScan.scan()
		#creates message header
		#self.msg.header.seq = sequence
		#self.msg.header.stamp = self.startTime
		#self.msg.header.frame_id = "/base_link"

		#pub.publish(self.msg)
		print pub.angle_min
		print pub.angle_max
		print pub.angle_increment
		print pub.time_increment
		print pub.scan_time
		print pub.range_min
		print pub.range_max
		print pub.ranges

		sequence = sequence+1


	#keeps the node from quitting
	#rospy.spin()