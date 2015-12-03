//laserScan Class Morgan Dykshorn
#include <Lidar.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include "BlackADC/BlackADC.h"
#include <SimpleGPIO.h>


#ifndef _laserScan_H_
#define _laserScan_H_


class laserScan
{
	public:
		laserScan();

		void calibrate(Lidar& , BlackLib::BlackADC& );
		void scan(Lidar& , BlackLib::BlackADC& );

		//void getRanges(float (&ranges)[400]);
		float ranges[];
		
	private:
		
		float angle;
		float threshold;
		float distance;
		int lastval;
		//float ranges[];
		
		void initialize_pins();
		void set_all_pins_low();
		void fullstep(int pin_index);
};
#endif