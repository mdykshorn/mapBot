//laserScan class
#include <Lidar.h>
#include <iostream>
#include <string>
//#include "BlackLib/BlackADC/BlackADC.h"
#include <SimpleGPIO.h>
#include "laserScan.h"
#include <unistd.h>

//default constructor
laserScan::laserScan()
{
	angle = 0.0;
	threshold = 570;
	distance = 0.0;
	lastval = 600;
	//float ranges[400] = {};
}
//need to figure out pin values
void laserScan::initialize_pins()
{
	//initializes pins
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
void laserScan::fullstep(int pin_index)
{
	int pins[] = {27, 11, 15, 12};
	gpio_set_value(pins[pin_index], HIGH);
	gpio_set_value(pins[(pin_index+3) % 4], HIGH);
	gpio_set_value(pins[(pin_index+1) % 4], LOW);
	gpio_set_value(pins[(pin_index+2) % 4], LOW);
}

float ranges[400] = {};

//public functions
void laserScan::calibrate(Lidar& lidar/*, BlackLib::BlackADC& analog*/)
{
	//sets angle to not be zero so loop will run
	laserScan::angle = 1.0;
	//initializes rotateVal, the value from the IR sensor
	int rotateVal = 0;
	//initializes the LIDARLite
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
			//rotateVal = analog.getNumericValue();
			if (laserScan::lastval < laserScan::threshold && rotateVal > laserScan::threshold)
			{
				laserScan::angle = 0;
			}
			laserScan::lastval = rotateVal;	
		}
	}
	laserScan::set_all_pins_low();
}	

void laserScan::scan(Lidar& lidar/*, BlackLib::BlackADC& analog*/)
{
	//initializes count
	int count = 0;
	int rotateVal = 0;
	//sets angle slightly above 0 so loop will run
	laserScan::angle = 0.001;
	//runs while there are less than 400 data points or the angle is not 0
	while(count<400 && laserScan::angle != 0)
	{
		for (int pin_index; pin_index++; pin_index<4)
		{
			//steps motor
			laserScan::fullstep(pin_index);
			//waits 10 ms
			usleep(10000);
			//checks the ADC sensor
			//rotateVal = analog.getNumericValue();
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
}
//gets range array
//not sure if most efficient way to accomplish this
/*
void laserScan::getRanges(float (&ranges)[400])
{
	for(int i; i++; i<400)
	{
		ranges[i] = laserScan::ranges[i];
	}
}
*/
