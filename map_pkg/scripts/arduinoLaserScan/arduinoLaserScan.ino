/* =============================================================================

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
  
  Lscan.calibrate(lidar);
}

void loop()
{

  Lscan.scan(lidar);

  for (int i = 0; i<400; i++)
  {
    Serial.println(Lscan.ranges[i]);
  }

}
