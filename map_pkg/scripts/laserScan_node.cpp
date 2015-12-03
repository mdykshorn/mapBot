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
#inlcude <laserScan.h>


int main()
{
  //initilizes node
  ros::init(argc, argv, "laserScan");

 
  ros::NodeHandle n;

  //creates am instance of the laserScan class
  laserScan::laserScan scan;
  
   //creates object of the message types
   std_msgs::Header header;
   sensor_msgs::LaserScan msg;
  
  //creates publisher
  ros::Publisher pub = n.advertise<sensor_msgs/LaserScan>("scan", 10);

  //calibrates sensor
  scan.calibrate();
	
  //adds constant message parameters
  msg.angle_increment = 0.015708;
  msg.range_min = 0;
  msg.range_max = 40;
  
  //initilizes necesary variables
  float startTime = 0.0;
  float endTime = 0.0;
  float scanTime = 0.0;
  float timeIncrement = 0.0;
	
	
  //a loop count to keep track of how many scans we have run
  int count = 0;
  while (ros::ok())
  {
	//records start time
	startTime = ros::Time::now();
	//runs scan
	scan.scan();
	//records end time
	endTime = ros::Time::now();
	
	scanTime = endTime - startTime;
	timeIncrement = scanTime/400;

	
	//get necessary message variables from laserScan
	//header
	msg.header.seq = count;
	msg.header.stamp.sec = startTime;
	msg.header.frame_id = "base_link"
	//body
	//hard codes min and max angle - actual value can deviate
	//by +-1 degree but it is negligible in the scan
	msg.angle_min = .0157077
	msg.angle_max = 6.28308
	//calculates time increment by dividing scan time by number of data points
	msg.time_increment = timeIncrement;
	msg.scan_time = scanTime;
	//get the range array
	scan.getRanges(&msg.ranges);

	
	//publishes message
    pub.publish(msg);

	//keeps loop running
    ros::spin();

	//increments count
    count++;
  }

  return 0;
}
