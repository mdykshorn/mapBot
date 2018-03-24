# mapBot
Inexpensive SLAM mapping robot

# Installation

Install ROS and all packages in khan_robot

# Connecting to the Beaglebone

in a terminal `ssh ubuntu@<beaglebone_ip>`

password: temppwd

# Setup ROS for multi master use

### On your computer
find the ip of your computer using `ifconfig` and add the following line to you `~/.bashrc`, with the ip you found
`export ROS_IP=<your_computer_ip>`

### on the beaglebone
in ssh session with beaglebone: `export ROS_MASTER_URI=http://<your_computer_ip>:11311`

# Launching
start `roscore` on your computer

### on beaglebone
This will launch the nodes that control the motors and read odometry info from the encoders

** must be in a sudo shell to run files **
1. type `sudo -s` to get a sudo shell
2. `cd mapBot`
3. `source devel/setup.bash`
4. `roslaunch general_khan general_khan.launch`

### on your computer
This will launch controllers for the robot

1. `cd mapBot`
2. `catkin_make`
3. `roslaunch khan_robot example_khan_hw.launch`

### Controlling the robot
the rover can be controlled by a `twist` msg on the topic `/cmd_vel`, the khan_robot package has various ways of publishing the message, including `rosjoy`, `rqt_robot_steering`, or command line publishing
