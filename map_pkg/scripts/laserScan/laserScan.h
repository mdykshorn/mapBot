//laserScan Class Morgan Dykshorn
#include <Lidar.h>
#include "ros/ros.h"
//#include <Stepper.h>

#ifndef _laserScan_H_
#define _laserScan_H_


class laserScan
{
	public:
		laserScan();

		void calibrate(Lidar& lidar, Stepper& motor);
		void scan(Lidar& lidar, Stepper& motor);
		float getAngle_min();
		float getAngle_max();
		float getTime_increment();
		float getScan_time();
		float getStart_time();
		void getRanges(&ranges[]);
		
		
	private:
		
		float angle;
		float threshold;
		float distance;
		float lastval;
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
		
		void set_all_pins_low();
};
#endif