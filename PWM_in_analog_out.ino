/* =============================================================================
  LIDAR-Lite v2: PWM operation

  This example demonstrates how to read measurements from LIDAR-Lite v2 "Blue
  Label" using PWM

  The library is in BETA, so subscribe to the github repo to recieve updates, or
  just check in periodically:
  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

unsigned long pulse_width;
int mappedDistance;

void setup()
{
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
}

void loop()
{
  pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
       // pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
  pulse_width = constrain(pulse_width, 0, 10000); //constrians values to be between 0 and 10000(0-10m)
  mappedDistance = map(pulse_width, 0, 10000, 0, 255);  //maps distance values to a pwm signal from 0-255
  analogWrite(9, mappedDistance);
  }
}
