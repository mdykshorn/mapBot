#!/usr/bin/python
# ROS node that reads a jointstate message and controls the motors
# Author: Morgan Dykshorn mdd27@vt.edu

# Import the Default ROS tools
import rospy
import Khan_IO
# Import the JointState message from sensor_msgs
from sensor_msgs.msg import JointState


#instatiatates the motorDrive class as an object
drive = Khan_IO.motorDrive()


#Function which is called every time a JointState is received, if the Subscriber is set up to use this function
def callFrontL(data):

	
	#gets velocity from JointState message
	vel = data.velocity[0]
	#converts rad/s to PWM value -100 to 100
	speed = (7.885288744*vel)
	#calls the left front function that drives at given speed
	drive.leftFront(speed)

def callRearL(data):

	#gets velocity from JointState message
	vel = data.velocity[0]
	#converts rad/s to PWM value -100 to 100
	speed = (7.885288744*vel)
	#calls the left front function that drives at given speed
	drive.leftRear(speed)

def callFrontR(data):

	#gets velocity from JointState message
	vel = data.velocity[0]
	#converts rad/s to PWM value -100 to 100
	speed = (7.885288744*vel)
	#calls the left front function that drives at given speed
	drive.rightFront(speed)

def callRearR(data):

	#gets velocity from JointState message
	vel = data.velocity[0]
	#converts rad/s to PWM value -100 to 100
	speed = (7.885288744*vel)
	#calls the left front function that drives at given speed
	drive.rightRear(speed)


# If this is loaded as the main python file, execute the main details
if __name__ == '__main__':
  try:
    #Initialize node
    rospy.init_node('motorcontrol')
	
    #Create subscriber, and tell it to call js_call() whenever a message is received
    rospy.Subscriber('/py_controller/front_left_wheel/cmd', JointState, callFrontL)
    rospy.Subscriber('/py_controller/rear_left_wheel/cmd', JointState, callRearL)	
    rospy.Subscriber('/py_controller/front_right_wheel/cmd', JointState, callFrontR)
    rospy.Subscriber('/py_controller/rear_right_wheel/cmd', JointState, callRearR)
	
    #We need to wait for new messages
    rospy.spin()
  #If we are interrupted, catch the exception, but do nothing
  except rospy.ROSInterruptException:
    pass