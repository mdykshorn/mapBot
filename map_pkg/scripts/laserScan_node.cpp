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
	
	
  //a loop count to keep track of how many scans we have run
  int count = 0;
  while (ros::ok())
  {

	//runs scan
	scan.scan();
	
	//get necesary message variables from laserScan
	//header
	msg.header.seq = count;
	msg.header.stamp.sec = scan.getStart_time;
	msg.header.frame_id = "base_link"
	//body
	msg.angle_min = scan.getAngle_min;
	msg.angle_max = scan.getAngle_max;
	msg.time_increment = scan.getTime_increment;
	msg.scan_time = scan.getScan_time;
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
