/*
 * =====================================================================================
 *
 *       Filename:  laserScan_node.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  12/2/2015
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Morgan Dykshorn <mdd27@vt.edu>
 *   Organization:  
 *
 * =====================================================================================
 */
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include <laserScan.h>
#include <Lidar.h>
#include "BlackLib/BlackADC/BlackADC.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

//needed when mixing c and c++ libraries
#ifdef __cplusplus
    extern "C" {
#endif
        #include "lsquaredc.h"

#ifdef __cplusplus
    }
#endif

int main(int argc, char **argv)
{
  //initializes node
  ros::init(argc, argv, "laserScan_node");
  ros::NodeHandle n;
  //creates publisher
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);


  //creates am instance of the laserScan class
  laserScan scan;
  Lidar lidar;
  BlackLib::BlackADC analog(BlackLib::AIN0);

  //creates object of the message types
  std_msgs::Header header;
  sensor_msgs::LaserScan msg;
  

  //calibrates sensor
  scan.calibrate(lidar, analog);
	
  //adds constant message parameters
  msg.angle_increment = 0.015708;
  msg.range_min = 0;
  msg.range_max = 40;
  
  //initializes necessary variables
  float startTime = 0.0;
  float endTime = 0.0;
  float scanTime = 0.0;
  float timeIncrement = 0.0;
  float range[400]={};
	
	
  //a loop count to keep track of how many scans we have run
  int count = 0;
  while (ros::ok())
  {
	//records start time
	startTime = ros::Time::now().toSec();
	//runs scan
	scan.scan(lidar, analog);
	//records end time
	endTime = ros::Time::now().toSec();
	
	scanTime = endTime - startTime;
	timeIncrement = scanTime/400;

	
	//get necessary message variables from laserScan
	//header
	msg.header.seq = count;
	msg.header.stamp.sec = startTime;
	msg.header.frame_id = "base_link";
	//body
	//hard codes min and max angle - actual value can deviate
	//by +-1 degree but it is negligible in the scan
	msg.angle_min = .0157077;
	msg.angle_max = 6.28308;
	//calculates time increment by dividing scan time by number of data points
	msg.time_increment = timeIncrement;
	msg.scan_time = scanTime;
	//get the range data
	for (int i; i++; i<400)
	{
		msg.ranges[i] = scan.ranges[i];
	}


	
	//publishes message
    pub.publish(msg);

	//keeps loop running
    ros::spin();

	//increments count
    count++;
  }

  return 0;
}
