/* =============================================================================
  LIDAR-Lite v2: Single sensor, get distance as fast as possible

  This example file demonstrates how to take distance measurements as fast as
  possible, when you first plug-in a LIDAR-Lite into an Arduino it runs 250
  measurements per second (250Hz). Then if we setup the sensor by reducing the
  aquisiton record count by 1/3 and incresing the i2c communication speed from
  100kHz to 400kHz we get about 500 measurements per second (500Hz). Now if we
  throttle the reference and preamp stabilization processes during the distance
  measurement process we can increase the number of measurements to about 750
  per second (750Hz).

   The library is in BETA, so subscribe to the github repo to recieve updates, or
   just check in periodically:
   https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

   To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>
#include <laserScan.h>


//create a new laserScan instance
laserScan Lscan;
LIDARLite lidar;


void setup()
{
  Serial.begin(115200);
  //sets up pins
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  //sets pins low
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  
  Lscan.calibrate();
}

void loop()
{
  
  Lscan.scan(lidar);

  for (int i = 0; i<400; i++)
  {
    Serial.println(Lscan.ranges[i];
  }

}
