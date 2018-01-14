#!/usr/bin/python
#imports necessary libraries
import rospy
import Adafruit_BBIO.ADC as ADC
import math
from sensor_msgs.msg import Range
from std_msgs.msg import Header


# Create variable so we can always see/use it, but set it to a value that indicates it's not yet valid
pub = None

def callDist(data):

		#reads voltage value
        voltage = ADC.read("P9_40")
		#converts voltage values into distance(meters)
        distance = (voltage**-.8271732796)
        distance = distance*.1679936709
    	#checks and discards data outside of accurate range
        if distance>2:
            distance = 2
        elif distance<.15:
            distance = .15

    	#Writes data to Range message
        msg = Range()
        header = Header()
        #creates header
        msg.header.stamp.secs = rospy.get_time()
        msg.header.frame_id = "/base_link"
        msg.radiation_type = 1
        msg.field_of_view = .15	#about 8.5 degrees
        msg.min_range = .15
        msg.max_range = 2
        msg.range = distance
        pub.publish(msg)


#creates a function named distance that reads the distance from the IR sensor and publishes it to Range
def distance():
	#initializes the node named distance
    rospy.init_node('irdistance', anonymous=True)
    # Declare we are using the pub defined above, not a new local variable
    global pub
    pub = rospy.Publisher('/range', Range, queue_size=10)
	#sets up BBIO ADC
    ADC.setup()
	#calls the callback every .1 seconds
    rospy.Timer(rospy.Duration(0.1), callDist)
	#keeps the node from quitting
    rospy.spin()
	
#runs the ir read function as long as there isn't an exception
if __name__ == '__main__':
	try:
		distance()
	except rospy.ROSInterruptException:
	    pass