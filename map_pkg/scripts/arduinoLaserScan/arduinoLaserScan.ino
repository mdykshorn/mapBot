/* =============================================================================

=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>
#include <laserScan.h>
#include <Stepper.h>


//create a new laserScan instance
laserScan Lscan;
LIDARLite lidar;
Stepper motor(200, 13, 12, 11, 10);

void setup()
{
  Serial.begin(115200);
  //sets up pins
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  //sets pins 8 and 9 high for PWM inputs
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);

  lidar.begin(1, true);
  motor.setSpeed(60);
  Lscan.calibrate(motor);
}

void loop()
{


  Lscan.scan(lidar, motor);
  for (int i = 0; i<400; i++)
  {
    Serial.println(Lscan.ranges[i]);
  }
}
