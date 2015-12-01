//laserScan class
#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <Stepper.h>
#include "laserScan.h"



//default constructor
laserScan::laserScan()
{
	float angle = 0.0;
	float threshold = 540;
	float distance = 0.0;
	float lastval = 600;
	float startTime = 0.0;
	float finishTime = 0.0;
	float datatime1 = 0.0;
	float datatime2 = 0.0;
	float angleR [400] = {};	
}

void laserScan::set_all_pins_low()
{
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
}

//public variables
float ranges [400] = {};
float angle_min = 0.0;
float angle_max = 0.0;
float timeIncrement = 0.0;
float scanTime = 0.0;

//public functions
void laserScan::calibrate(Stepper& motor)
{
	//sets angle to not be zero so loop will run
	laserScan::angle = 1.0;
	float rotateVal = 0.0;
	laserScan::lastval = 600;
	laserScan::threshold = 570;
	
	while (laserScan::angle!=0)
	{
		//moves stepper
		motor.step(1);
		//waits
		//delay(2.5);
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

void laserScan::scan(LIDARLite& lidar, Stepper& motor)
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
		if (count > 350 && laserScan::lastval < laserScan::threshold && rotateVal > laserScan::threshold)
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
		//Serial.println(lidar.distance());
		laserScan::ranges[count] = lidar.distance();
			
		laserScan::timeIncrement = (laserScan::datatime1 - laserScan::datatime2);
		//laserScan::ranges[count] = laserScan::distance;
		laserScan::datatime2 = laserScan::datatime1;
		count++;
	}
	laserScan::set_all_pins_low();
	//laserScan::angle_min = (laserScan::angleR[0]*0.0174533);
	
	//laserScan::angle_max = (laserScan::angle_max*0.0174533);
	
	//laserScan::finishTime = millis();
	
	//laserScan::scanTime = (laserScan::finishTime - laserScan::startTime);
	

}
