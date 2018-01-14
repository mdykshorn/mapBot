#!/usr/bin/python
# ROS node that reads quadrature encoders and outputs a jointstate message
# Author: Morgan Dykshorn mdd27@vt.edu

# Import the Default ROS tools
import rospy
import Adafruit_BBIO.GPIO as GPIO
import quadrature
# Import the JointState message from sensor_msgs
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


#creates an object of QuadratureEstimatior for the right encoder
quadCalcR = quadrature.QuadratureEstimator()
#creates an object of QuadratureEstimatior for the left encoder
quadCalcL = quadrature.QuadratureEstimator()

# Create variable so we can always see/use it, but set it to a value that indicates it's not yet valid
pubFR = None
pubRR = None
pubFL = None
pubRL = None

def callLeft(data):
	#gets time
    time = rospy.get_time()
	#reads encoder inputs
    channelA = GPIO.input("P9_23")
    channelB = GPIO.input("P9_30")

	#puts inputs into the quadrature decoder program
    quadCalcL.update(channelA, channelB, time)
	
	#creates messages
    leftFMSG = JointState()
    leftRMSG = JointState()
    header = Header()
	
    #appends velocity and position data to the joinstate message
	#front wheel
    leftFMSG.header.stamp.secs = rospy.get_time()
    leftFMSG.name.append('front_left_wheel')
    leftFMSG.position.append(quadCalcL.position)
	#rear wheel
    leftRMSG.header.stamp.secs = rospy.get_time()
    leftRMSG.name.append('rear_left_wheel')
    leftRMSG.position.append(quadCalcL.position)
	
	#publishes message
    pubFL.publish(leftFMSG)
    pubRL.publish(leftRMSG)
	
def callRight(data):
	#gets time
    time = rospy.get_time()
	#reads encoder inputs
    channelA = GPIO.input("P8_17")
    channelB = GPIO.input("P8_26")

	#puts inputs into the quadrature decoder program
    quadCalcR.update(channelA, channelB, time)
	 
	#creates messages
    rightFMSG = JointState()
    rightRMSG = JointState()
    header = Header()

    #appends velocity and position data to the joinstate message
	#front wheel
    rightFMSG.header.stamp.secs = rospy.get_time()
    rightFMSG.name.append('front_right_wheel')
    rightFMSG.position.append(quadCalcL.position)
	#rear wheel
    rightRMSG.header.stamp.secs = rospy.get_time()
    rightRMSG.name.append('rear_right_wheel')
    rightRMSG.position.append(quadCalcL.position)

	#publishes message
    pubFR.publish(rightFMSG)
    pubRR.publish(rightRMSG)



# If this is loaded as the main python file, execute the main details
if __name__ == '__main__':
  try:
    #Initialize node
    rospy.init_node('encoders')
    #Create publisher, to send out a String with the first joint name of every received message as an example.
    pubFR = rospy.Publisher('/py_controller/front_right_wheel/encoder', JointState, queue_size=10)
    pubRR = rospy.Publisher('/py_controller/rear_right_wheel/encoder', JointState, queue_size=10)
    pubFL = rospy.Publisher('/py_controller/front_left_wheel/encoder', JointState, queue_size=10)
    pubRL = rospy.Publisher('/py_controller/rear_left_wheel/encoder', JointState, queue_size=10)
    
	#sets up GPIO channels
    GPIO.setup("P9_23", GPIO.IN)
    GPIO.setup("P9_30", GPIO.IN)
    GPIO.setup("P8_17", GPIO.IN)
    GPIO.setup("P8_26", GPIO.IN)	
	
    GPIO.add_event_detect("P9_23", GPIO.BOTH)
    GPIO.add_event_detect("P9_30", GPIO.BOTH)
    GPIO.add_event_detect("P8_17", GPIO.BOTH)
    GPIO.add_event_detect("P8_26", GPIO.BOTH)

    GPIO.add_event_callback("P9_23", callLeft)
    GPIO.add_event_callback("P9_30", callLeft)
    GPIO.add_event_callback("P8_17", callRight)
    GPIO.add_event_callback("P8_26", callRight)
 

    rospy.spin()
  #If we are interrupted, catch the exception, but do nothing
  except rospy.ROSInterruptException:
    pass