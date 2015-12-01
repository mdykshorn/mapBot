//laserScan Class Morgan Dykshorn
#include <Arduino.h>
#include <LIDARLite.h>
#include <Stepper.h>

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
		
		void calibrate(Stepper& motor);
		void scan(LIDARLite& lidar, Stepper& motor);
		
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
		
		void set_all_pins_low();
};
#endif