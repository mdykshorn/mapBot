cmake_minimum_required(VERSION 2.8.3)
project(map_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs)

catkin_package(
   INCLUDE_DIRS 
#  LIBRARIES mpu9150
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Alternatively we can use the CMakeLists.txt of the subdirs
#add_subdirectory(src/linux-mpu9150)

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
  scripts/laserScan
  scripts/laserScan/BlackLib
  scripts/laserScan/BlackLib/BlackADC
  scripts/laserScan/GPIO
  scripts/laserScan/i2c
)

#link_directories()


# The MPU9150 node
# Adds flags to the compiler command line for sources in the current directory and below.
add_definitions(-DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY)
add_executable(laserScan_node scripts/laserScan_node.cpp)
add_library(laserScan SHARED scripts/laserScan/laserScan.cpp)
add_library(Lidar SHARED scripts/laserScan/Lidar.cpp)
add_library(BlackLib SHARED scripts/laserScan/BlackLib/BlackCore.cpp)
add_library(BlackADC SHARED scripts/laserScan/BlackLib/BlackADC/BlackADC.cpp)
add_library(i2c SHARED scripts/laserScan/i2c/lsquaredc.c)
add_library(GPIO SHARED scripts/laserScan/GPIO/SimpleGPIO.cpp)
target_link_libraries(laserScan_node laserScan Lidar BlackLib BlackADC i2c GPIO  ${catkin_LIBRARIES})
#target_link_libraries(mpu9150_node ${catkin_LIBRARIES})

# imu utility
#add_executable(imu src/linux-mpu9150/imu.c)
#target_link_libraries(imu vector3d linux_glue mpu9150 inv_mpu  quaternion inv_mpu_dmp_motion_driver)

# imucal utility (calibration utility)
#add_executable(imucal src/linux-mpu9150/imucal.c)
#target_link_libraries(imucal linux_glue mpu9150 inv_mpu quaternion inv_mpu_dmp_motion_driver vector3d ${catkin_LIBRARIES})

install(TARGETS laserScan_node laserScan Lidar BlackLib BlackADC i2c GPIO
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(FILES src/accelcal.txt src/magcal.txt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY scripts/laserScan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
