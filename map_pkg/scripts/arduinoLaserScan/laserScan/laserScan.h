//laserScan Class Morgan Dykshorn
#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>

class LIDARLite;

class laserScan
{
	public:
		laserScan();

		float ranges;
		float angle_min;
		float angle_max;
		float timeIncrement;
		float scanTime;
		
		void calibrate();
		void scan(lidar);
		
	private:
	
		LIDARLite lidar;
		
		int pin_index;
		//sets pins
		int pins;
		float angle;
		float threshold;
		float distance;
		float lastval;
		
		float startTime;
		float finishTime;
		float datatime1;
		float datatime2;
		
		float angleR;
	
		void wavedrive(int pins, int pin_index);
		void fullstep(int pins, int pin_index);
};