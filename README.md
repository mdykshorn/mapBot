# mapBot
Inexpensive SLAM mapping robot

Repository Contains 2 packages

bb_mpu9150 is a package origianlly created by Víctor Mayoral Vilches
The bb_mpu9150 package takes data from an mpu_9150 IMU and publishes to the rostopic /IMU_euler
The package has been modified to publish with the ROS IMU message type

map_pkg is a package created specifically for our robot. The package contains a node
that spins our Lidar lite 360 degrees and publishes the scan in the ROS laserscan message type
