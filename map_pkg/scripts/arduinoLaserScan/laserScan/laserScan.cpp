//laserScan class
#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>
#include "laserScan.h"

//private functions and variables

int pin_index = 0;
int pins = [13, 12, 11, 10];
float angle = 0.0;
float threshold = .2;
float distance = 0.0;
float lastval = 1;
float startTime = 0.0;
float finishTime = 0.0;
float datatime1 = 0.0;
float datatime2 = 0.0;

float angleR = [400];

void laserScan::wavedrive(int pins, int pin_index)
{
	for i in range(len(pins)):
		if (i == pin_index)
		{
			digitalWrite(pins[i], HIGH)
		}
		else:
		{
			digitalWrite(pins[i], LOW)
		}
}

void laserScan::fullstep(int pins, int pin_index)
{
	digitalWrite(pins[pin_index], HIGH);
	digitalWrite(pins[pin_index+3 % 4], HIGH);
	digitalWrite(pins[pin_index+1 % 4], LOW);
	digitalWrite(pins[pin_index+2 % 4], LOW);
}

//creates object of LIDARlite class
LIDARLite lidar;

//default constructor
laserScan::laserScan()
{
	
	lidar.begin(1, true);
	
}


//public variables
float ranges = [400];
float angle_min = 0.0;
float angle_max = 0.0;
float timeIncrement = 0.0;
float scanTime = 0.0;

//public functions
void laserScan::calibrate()
{
	//sets angle to not be zero so loop will run
	laserScan::angle = 1.0;
	float rotateVal = 0.0;
	
	while (laserScan::angle!=0)
	{
		for (pin_index in range(len(laserScan::pins)))
		{	
			//moves stepper
			laserScan::fullstep(laserScan::pins, laserScan::pin_index);
			//waits
			delay(10)
			//checks the ADC sensor
			rotateVal = analogRead(0);
			if (rotateVal > laserScan::threshold && laserScan::lastval < laserScan::threshold)
			{
				laserScan::angle = 0;
			}
			laserScan::lastval = rotateVal;	
		}
	}
	laserScan::set_all_pins_low(laserScan::pins);
}	

void laserScan::scan(lidar)
{
	//initilizes count
	int count = 0;
	float rotateVal = 0.0;
	//sets angle slightly above 0 so loop will run
	laserScan::angle = .0001;
	//records start time
	laserScan::startTime = millis();
	//runs while there are less than 400 data points or the angle is not 0
	while(count<400 && laserScan::angle != 0)
	{
		for (pin_index in range(len(laserScan::pins)))
		{	
			laserScan::datatime1 = millis();
			//moves stepper
			laserScan::fullstep(laserScan::pins, laserScan::pin_index);
			//waits
			delay(10);
			//checks the ADC sensor
			rotateVal = analogRead(0);
			if (rotateVal > laserScan::threshold && laserScan::lastval < laserScan::threshold)
			{
				laserScan::angle_max - laserScan::angle;
				laserScan::angle = 0;
			}
			else:
			{
				laserScan::angle = laserScan::angle + .9;
				laserScan::angle_max = laserScan::angle;
			}
			laserScan::lastval = rotateVal;
			laserScan::angleR[count] = laserScan::angle;
			
			//read LIDARLite
			laserScan::distance = lidar.distance();
			
			
			laserScan::timeIncrement = (laserScan::datatime1 - laserScan::datatime2);
			laserScan::ranges[count] = laserScan::distance;
			laserScan::datatime2 = laserScan::datatime1;
			count = count+1;
		}
	}
	laserScan::set_all_pins_low(laserScan::pins);
	
	laserScan::angle_min = (laserScan::angleR[0]*0.0174533);
	
	laserScan::angle_max = (laserScan::angle_max*0.0174533);
	
	laserScan::finishTime = millis();
	
	laserScan::scanTime = (laserScan::finishTime - laserScan::startTime);
	

}
