#!/usr/bin/python

import Adafruit_BBIO.GPIO as GPIO



def callLeft(data):

	#reads encoder inputs
    channelA = GPIO.input("P9_23")
    channelB = GPIO.input("P9_30")

    print channelA, channelB

	#sets up GPIO channels
GPIO.setup("P9_23", GPIO.IN)
GPIO.setup("P9_30", GPIO.IN)
GPIO.setup("P8_17", GPIO.IN)
GPIO.setup("P8_26", GPIO.IN)	
	
	
#creates an event detect for each of the encoders 
GPIO.add_event_detect("P9_23", GPIO.BOTH)
GPIO.add_event_detect("P9_30", GPIO.BOTH)

while 1:


	#creates a callback if a change happens
	GPIO.add_event_callback("P9_23", callLeft)
	GPIO.add_event_callback("P9_30", callLeft)