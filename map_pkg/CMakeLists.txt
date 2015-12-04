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
  src/laserScan
  src/laserScan/BlackLib
  src/laserScan/GPIO
  src/laserScan/i2c
)

#link_directories()


# The MPU9150 node
# Adds flags to the compiler command line for sources in the current directory and below.
add_definitions(-DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY)
add_executable(laserScan_node src/laserScan_node.cpp)
add_library(laserScan SHARED src/laserScan/laserScan.cpp)
add_library(Lidar SHARED src/laserScan/Lidar.cpp)
add_library(BlackLib SHARED src/laserScan/BlackLib/BlackCore.cpp)
add_library(lsquaredc SHARED src/laserScan/i2c/lsquaredc.c)
add_library(SimpleGPIO SHARED src/laserScan/GPIO/SimpleGPIO.cpp)
target_link_libraries(laserScan_node laserScan Lidar BlackLib lsquaredc SimpleGPIO  ${catkin_LIBRARIES})
#target_link_libraries(mpu9150_node ${catkin_LIBRARIES})

# imu utility
#add_executable(imu src/linux-mpu9150/imu.c)
#target_link_libraries(imu vector3d linux_glue mpu9150 inv_mpu  quaternion inv_mpu_dmp_motion_driver)

# imucal utility (calibration utility)
#add_executable(imucal src/linux-mpu9150/imucal.c)
#target_link_libraries(imucal linux_glue mpu9150 inv_mpu quaternion inv_mpu_dmp_motion_driver vector3d ${catkin_LIBRARIES})

install(TARGETS laserScan_node laserScan Lidar BlackLib lsquaredc SimpleGPIO
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(FILES src/accelcal.txt src/magcal.txt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY src/ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
