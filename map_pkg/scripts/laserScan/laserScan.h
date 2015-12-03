//laserScan Class Morgan Dykshorn
#include <Lidar.h>
#include <unistd.h>
#include "ros/ros.h"
#include "BlackADC.h"
#include "SimpleGPIO.h"


#ifndef _laserScan_H_
#define _laserScan_H_


class laserScan
{
	public:
		laserScan();

		void calibrate(Lidar& lidar, BlackLib::BlackADC& analog(BlackLib::AIN0));
		void scan(Lidar& lidar, BlackLib::BlackADC& analog(BlackLib::AIN0);
		float getAngle_min();
		float getAngle_max();
		float getTime_increment();
		float getScan_time();
		float getStart_time();
		void getRanges(float &ranges[]);
		
		
	private:
		
		float angle;
		float threshold;
		float distance;
		int lastval;
		float startTime;
		float finishTime;
		float datatime1;
		float datatime2;
		float angleR[];
		float angle_min;
		float angle_max;
		float timeIncrement;
		float scanTime;
		float ranges[];
		
		void initialize_pins();
		void set_all_pins_low();
		void fullstep(int pin_index);
};
#endif