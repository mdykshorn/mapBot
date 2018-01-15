#!/usr/bin/python
# ROS node that reads quadrature encoders and outputs a jointstate message
# Author: Morgan Dykshorn mdd27@vt.edu

# Import the Default ROS tools
import rospy
import ctypes
# Import the JointState message from sensor_msgs
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import UInt64
from math import pi, floor
from collections import deque

ticks_per_revolution = 333

radians_per_tick = (2.0 * pi) / ticks_per_revolution

timesL = deque(maxlen = 5)
timesR = deque(maxlen = 5)

prev_timeL = None
prev_timeR = None

prev_posL = None
prev_posR = None

# Create variable so we can always see/use it, but set it to a value that indicates it's not yet valid
pubFR = None
pubRR = None
pubFL = None
pubRL = None

velocity = None

def callLeft(data):
    global prev_timeL
    global prev_posL
	#gets time
    time = rospy.get_time()

    signed_num = ctypes.c_long(data.data).value

    #calc position
    position = signed_num - prev_posL
    prev_posL = signed_num

    # Update position
    theta = signed_num * radians_per_tick
    theta_vel = position * radians_per_tick
    # Store position, wrapped as [-pi, pi] for ease of use
    #new_position = theta - (2.0 * pi) * floor((theta + pi) / (2.0 * pi))
    new_position = theta

    timesL.append((theta_vel, time - prev_timeL))

    # Sum up position offset and how long those updates have taken
    counter = 0
    avg_time = 0
    # For every velocity sample in the circular buffer
    for ii, jj in list(timesL):
      counter = counter + ii
      avg_time = avg_time + jj

    # Compute velocity, but can't divide by zero (or negative time)
    if avg_time >= 1.0e-3:
      velocity = counter/ avg_time
    else:
      velocity = 0

    prev_timeL = time

	#creates messages
    leftFMSG = JointState()
    leftRMSG = JointState()
    header = Header()

    #appends velocity and position data to the joinstate message
	#front wheel
    leftFMSG.header.stamp.secs = rospy.get_time()
    leftFMSG.name.append('front_left_wheel')
    leftFMSG.position.append(new_position)
    leftFMSG.velocity.append(velocity)
	#rear wheel
    leftRMSG.header.stamp.secs = rospy.get_time()
    leftRMSG.name.append('rear_left_wheel')
    leftRMSG.position.append(new_position)
    leftRMSG.velocity.append(velocity)

	#publishes message
    pubFL.publish(leftFMSG)
    pubRL.publish(leftRMSG)

def callRight(data):
    global prev_timeR
    global prev_posR
	#gets time
    time = rospy.get_time()

    signed_num = ctypes.c_long(data.data).value

    #calc position
    position = signed_num - prev_posR
    prev_posR = signed_num

    # Update position
    theta = signed_num * radians_per_tick
    theta_vel = position * radians_per_tick
    # Store position, wrapped as [-pi, pi] for ease of use
    #new_position = theta - (2.0 * pi) * floor((theta + pi) / (2.0 * pi))
    new_position = theta

    timesL.append((theta_vel, time - prev_timeR))

    # Sum up position offset and how long those updates have taken
    counter = 0
    avg_time = 0
    # For every velocity sample in the circular buffer
    for ii, jj in list(timesR):
      counter = counter + ii
      avg_time = avg_time + jj

    # Compute velocity, but can't divide by zero (or negative time)
    if avg_time >= 1.0e-3:
      velocity = counter/ avg_time
    else:
      velocity = 0

    prev_timeR = time


	#creates messages
    rightFMSG = JointState()
    rightRMSG = JointState()
    header = Header()

    #appends velocity and position data to the joinstate message
	#front wheel
    rightFMSG.header.stamp.secs = rospy.get_time()
    rightFMSG.name.append('front_right_wheel')
    rightFMSG.position.append(new_position)
    rightFMSG.velocity.append(velocity)
	#rear wheel
    rightRMSG.header.stamp.secs = rospy.get_time()
    rightRMSG.name.append('rear_right_wheel')
    rightRMSG.position.append(new_position)
    rightRMSG.velocity.append(velocity)

	#publishes message
    pubFR.publish(rightFMSG)
    pubRR.publish(rightRMSG)



# If this is loaded as the main python file, execute the main details
if __name__ == '__main__':
  try:
    prev_timeL = 0
    prev_timeR = 0
    prev_posL = 0
    prev_posR = 0
    #Initialize node
    rospy.init_node('encoders')
    #Create publisher, to send out a String with the first joint name of every received message as an example.
    pubFR = rospy.Publisher('/py_controller/front_right_wheel/encoder', JointState, queue_size=10)
    pubRR = rospy.Publisher('/py_controller/rear_right_wheel/encoder', JointState, queue_size=10)
    pubFL = rospy.Publisher('/py_controller/front_left_wheel/encoder', JointState, queue_size=10)
    pubRL = rospy.Publisher('/py_controller/rear_left_wheel/encoder', JointState, queue_size=10)

    rospy.Subscriber("leftEncoder", UInt64, callLeft)
    rospy.Subscriber("rightEncoder", UInt64, callRight)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


  #If we are interrupted, catch the exception, but do nothing
  except rospy.ROSInterruptException:
    pass
