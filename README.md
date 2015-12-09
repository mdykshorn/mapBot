# mapBot
Inexpensive SLAM mapping robot

Repository Contains 2 packages

map_pkg is a package created specifically for our robot. The package contains a node
that spins our Lidar lite 360 degrees and publishes the scan in the ROS laserscan message type.

**map_pkg currently contains 2 versions of the laserScan_node, the first and currently used version is in python
and doesn't support i2c for the LidarLite, the second is written in c++ but is not completely working, the c++ version does
support i2c**

The second package is for configuration files and launch files for the navigation stack

CAD files for our rotating assembly and lasercut pices are also availible in the CAD folder
