//laserScan class
#include <LIDARLite.h>
#include "ros/ros.h"
//#include <Stepper.h>
#include "laserScan.h"


//default constructor
laserScan::laserScan()
{
	float laserScan::angle = 0;
	float laserScan::threshold = 570;
	float laserScan::distance = 0;
	float laserScan::lastval = 600;
	float laserScan::startTime = 0;
	float laserScan::finishTime = 0;
	float laserScan::datatime1 = 0;
	float laserScan::datatime2 = 0;
	float laserScan::angleR [400] = {};
	float laserScan::angle_min = 0;
	float laserScan::angle_max = 0;
	float laserScan::timeIncrement = 0;
	float laserScan::scanTime = 0;
	float laserScan::ranges[400] = {};
}

//Needs Updating to Beaglebone pin values
void laserScan::set_all_pins_low()
{
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
}

//public functions
void laserScan::calibrate(Lidar& lidar, Stepper& motor)
{
	//sets angle to not be zero so loop will run
	laserScan::angle = 1.0;
	//initilizes rotateVal, the value from the IR sensor
	float rotateVal = 0.0;
	//initilizes the LIDARLite
	lidar.begin(1);
	
	while (laserScan::angle!=0)
	{
		//need to fix stepper functions
		//moves stepper
		motor.step(1);
		//waits
		//delay(2.5);
		//need to implement adc read for Beaglebone
		//checks the ADC sensor
		rotateVal = analogRead(5);
		if (laserScan::lastval < laserScan::threshold && rotateVal > laserScan::threshold)
		{
			laserScan::angle = 0;
		}
		laserScan::lastval = rotateVal;	
	}
	laserScan::set_all_pins_low();
}	

//function needs to pass in ros time
void laserScan::scan(Lidar& lidar, Stepper& motor, )
{
	//initializes count
	int count = 0;
	float rotateVal = 0.0;
	//sets angle slightly above 0 so loop will run
	laserScan::angle = 0.001;
	//records start time
	laserScan::startTime = millis();
	//runs while there are less than 400 data points or the angle is not 0
	while(count<400 && laserScan::angle != 0)
	{

		laserScan::datatime1 = millis();
		//moves stepper
		motor.step(1);
		//waits
		delay(2.5);
		//checks the ADC sensor
		rotateVal = analogRead(5);
		if (laserScan::lastval < laserScan::threshold && rotateVal > laserScan::threshold)
		{
			laserScan::angle_max = laserScan::angle;
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
		laserScan::ranges[count] = lidar.distance();
			
		laserScan::timeIncrement = (laserScan::datatime1 - laserScan::datatime2);
		laserScan::datatime2 = laserScan::datatime1;
		count++;
	}
	laserScan::set_all_pins_low();
	laserScan::angle_min = (laserScan::angleR[0]*0.0174533);
	
	laserScan::angle_max = (laserScan::angle_max*0.0174533);
	
	laserScan::finishTime = millis();
	
	laserScan::scanTime = (laserScan::finishTime - laserScan::startTime);
}
//getter functions for returning variables
float laserScan::getAngle_min()
{
	return laserScan::angle_min;
}

float laserScan::getAngle_max()
{
	return laserScan::angle_max;
}

float laserScan::getTime_increment()
{
	return laserScan::timeIncrement;
}

float laserScan::getScan_time()
{
	return laserScan::scanTime;
}

float laserScan::getStart_time()
{
	return laserScan::startTime;
}

//not sure if most efficient way to accomplish this
void laserScan::getRanges(&ranges[])
{
	for(int i; i++; i<400)
	{
		ranges[i] = laserScan::ranges[i]
	}
}
