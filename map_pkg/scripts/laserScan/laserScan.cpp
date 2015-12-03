//laserScan class
#include <Lidar.h>
#include "ros/ros.h"
#include "BlackADC.h"
#include "SimpleGPIO.h"
#include "laserScan.h"
#include <unistd.h>


//default constructor
laserScan::laserScan()
{
	float angle = 0.0;
	float threshold = 570;
	float distance = 0.0;
	int lastval = 600;
	float startTime = 0.0;
	float finishTime = 0.0;
	float datatime1 = 0.0;
	float datatime2 = 0.0;
	float angle_min = 0.0;
	float angle_max = 0.0;
	float timeIncrement = 0.0;
	float scanTime = 0.0;
	float ranges[400] = {};
}
//need to figure out pin values
void laserScan::initialize_pins()
{
	//initilizes pins
	gpio_omap_mux_setup("mcasp0_fsr ", "07"); 	//gpio  P9pin# 27
	gpio_omap_mux_setup("gpmc_ad15", "07"); //gpio 26 P8pin#15
	gpio_omap_mux_setup("gpmc_ad13", "07"); 	//gpio 38 P8pin# 11
	gpio_omap_mux_setup("gpmc_ad12", "07"); 	//gpio 34 P8pin# 12
}

//Needs confirmation of correct function use
void laserScan::set_all_pins_low()
{
	//sets all pins low
	gpio_set_value(27, LOW);
	gpio_set_value(15, LOW);
	gpio_set_value(11, LOW);
	gpio_set_value(12, LOW);
}

//steps pins
//Needs confirmation of correct function use
void fullstep(int pin_index)
{
	int pins[] = {27, 11, 15, 12};
	gpio_set_value(pins[pin_index], HIGH);
	gpio_set_value(pins[(pin_index+3) % 4], HIGH);
	gpio_set_value(pins[(pin_index+1) % 4], LOW);
	gpio_set_value(pins[(pin_index+2) % 4], LOW);
}

//public functions
void laserScan::calibrate(Lidar& lidar, BlackLib::BlackADC& analog(BlackLib::AIN0))
{
	//sets angle to not be zero so loop will run
	laserScan::angle = 1.0;
	//initilizes rotateVal, the value from the IR sensor
	int rotateVal = 0;
	//initilizes the LIDARLite
	lidar.begin(1);
	
	while (laserScan::angle!=0)
	{
		for (int pin_index; pin_index++; pin_index<4)
		{
			//steps motor
			laserScan::fullstep(pin_index);
			//waits
			usleep(2500);
	
			//checks the ADC sensor
			rotateVal = analog.getNumericValue();
			if (laserScan::lastval < laserScan::threshold && rotateVal > laserScan::threshold)
			{
				laserScan::angle = 0;
			}
			laserScan::lastval = rotateVal;	
		}
	}
	laserScan::set_all_pins_low();
}	

void laserScan::scan(Lidar& lidar, BlackLib::BlackADC& analog(BlackLib::AIN0))
{
	//initializes count
	int count = 0;
	int rotateVal = 0;
	//sets angle slightly above 0 so loop will run
	laserScan::angle = 0.001;
	//records start time
	//laserScan::startTime = ros::Time::now();
	//runs while there are less than 400 data points or the angle is not 0
	while(count<400 && laserScan::angle != 0)
	{
		for (int pin_index; pin_index++; pin_index<4)
		{
			//steps motor
			laserScan::fullstep(pin_index);
			//waits
			usleep(10000);
			//checks the ADC sensor
			rotateVal = analog.getNumericValue();
			if (laserScan::lastval < laserScan::threshold && rotateVal > laserScan::threshold)
			{
				laserScan::angle = 0;
			}
			else
			{
				laserScan::angle = laserScan::angle + .9;
			}
			laserScan::lastval = rotateVal;

			//read LIDARLite
			laserScan::ranges[count] = lidar.distance();
			
			//increments count
			count++;
		}
	}
	laserScan::set_all_pins_low();
	//hardcodes min and max angle - actual value can deviate by +-1 degree but it is negligible in the scan
	laserScan::angle_min = (.0157077);
	laserScan::angle_max = (6.28308);
	
	//laserScan::finishTime = ros::Time::now();
	
	laserScan::scanTime = (laserScan::finishTime - laserScan::startTime);
	//caluclates time between data points
	laserScan::timeIncrement = laserScan::scanTime/count;
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
void laserScan::getRanges(float &ranges[])
{
	for(int i; i++; i<400)
	{
		ranges[i] = laserScan::ranges[i]
	}
}
