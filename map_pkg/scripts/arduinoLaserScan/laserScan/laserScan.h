//laserScan Class Morgan Dykshorn
#include <Arduino.h>
#include <LIDARLite.h>

#ifndef _laserScan_H_
#define _laserScan_H_


class laserScan
{
	public:
		laserScan();

		float ranges[];
		float angle_min;
		float angle_max;
		float timeIncrement;
		float scanTime;
		
		void calibrate(LIDARLite& lidar);
		void scan(LIDARLite& lidar);
		
	private:
		

		//sets pins
		int pins[];
		float angle;
		float threshold;
		float distance;
		float lastval;
		
		float startTime;
		float finishTime;
		float datatime1;
		float datatime2;
		
		float angleR[];
	
		void set_all_pins_low();
		void fullstep(int pins[], int pin_index);
};
#endif