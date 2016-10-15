#!/usr/bin/python
import Adafruit_BBIO.ADC as ADC
from time import sleep

ADC.setup()

runtill = 1
count = 0
distance = []
Value = []


while runtill==1:
	avgvalue = 0
	distance.append(input('What is the distance?(hold distance constant for 3 seconds to get reading)'))
	#pauses for 1 second
	sleep(1)

	#averages 10 values taken in a short period of time
	for x in range(0,10):
		valueadd = ADC.read("P9_40")
		print (valueadd)
		avgvalue = avgvalue+valueadd

	avgvalue = avgvalue/10

	#sets the average value to the value in the array
	Value.append(avgvalue)

	#runs the loop until the user exits
	runtill = input('Enter 1 to continue or 0 to stop')
	count = count+1

i = 0

print ('Distance        Value/n')
while i<count:
	
	print ('{0}              {1}/n'.format(distance[i],Value[i]))

	i=i+1



