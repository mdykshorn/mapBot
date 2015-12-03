//laserScan Class Morgan Dykshorn
#include <Lidar.h>
#include <unistd.h>
#include "BlackADC.h"
#include "SimpleGPIO.h"


#ifndef _laserScan_H_
#define _laserScan_H_


class laserScan
{
	public:
		laserScan();

		void calibrate(Lidar& lidar, BlackLib::BlackADC& analog(BlackLib::AIN0));
		void scan(Lidar& lidar, BlackLib::BlackADC& analog(BlackLib::AIN0));

		void getRanges(float &ranges[]);
		
		
	private:
		
		float angle;
		float threshold;
		float distance;
		int lastval;
		float ranges[];
		
		void initialize_pins();
		void set_all_pins_low();
		void fullstep(int pin_index);
};
#endif