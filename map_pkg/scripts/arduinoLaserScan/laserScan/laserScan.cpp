//laserScan class
#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>
#include "laserScan.h"


//default constructor
laserScan::laserScan()
{
	
}

//private functions and variables

int pins [4] = {13, 12, 11, 10};
float angle = 0.0;
float threshold = .2;
float distance = 0.0;
float lastval = 1;
float startTime = 0.0;
float finishTime = 0.0;
float datatime1 = 0.0;
float datatime2 = 0.0;

float angleR [400] = {};

void laserScan::set_all_pins_low()
{
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
}

void laserScan::fullstep(int pins[], int pin_index)
{
	digitalWrite(pins[pin_index], HIGH);
	digitalWrite(pins[pin_index+3 % 4], HIGH);
	digitalWrite(pins[pin_index+1 % 4], LOW);
	digitalWrite(pins[pin_index+2 % 4], LOW);
}



//public variables
float ranges [400] = {};
float angle_min = 0.0;
float angle_max = 0.0;
float timeIncrement = 0.0;
float scanTime = 0.0;

//public functions
void laserScan::calibrate(const LIDARLite lidar)
{
	//initilizes the lidar
	lidar.begin(1, true);
	//sets angle to not be zero so loop will run
	laserScan::angle = 1.0;
	float rotateVal = 0.0;
	
	while (laserScan::angle!=0)
	{
		for (int pin_index = 0; pin_index++; pin_index<4)
		{	
			//moves stepper
			laserScan::fullstep(laserScan::pins, pin_index);
			//waits
			delay(10);
			//checks the ADC sensor
			rotateVal = analogRead(0);
			if (rotateVal > laserScan::threshold && laserScan::lastval < laserScan::threshold)
			{
				laserScan::angle = 0;
			}
			laserScan::lastval = rotateVal;	
		}
	}
	laserScan::set_all_pins_low();
}	

void laserScan::scan(const LIDARLite lidar)
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
		for (int pin_index = 0; pin_index++; pin_index<4)
		{	
			laserScan::datatime1 = millis();
			//moves stepper
			laserScan::fullstep(laserScan::pins, pin_index);
			//waits
			delay(10);
			//checks the ADC sensor
			rotateVal = analogRead(0);
			if (rotateVal > laserScan::threshold && laserScan::lastval < laserScan::threshold)
			{
				laserScan::angle_max - laserScan::angle;
				laserScan::angle = 0;
			}
			else
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
	laserScan::set_all_pins_low();
	
	laserScan::angle_min = (laserScan::angleR[0]*0.0174533);
	
	laserScan::angle_max = (laserScan::angle_max*0.0174533);
	
	laserScan::finishTime = millis();
	
	laserScan::scanTime = (laserScan::finishTime - laserScan::startTime);
	

}
