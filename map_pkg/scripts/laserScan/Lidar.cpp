/* =============================================================================
  Lidar Arduino Library:

  The purpose of this library is two-fold:
  1.  Quick access all the basic functions of LIDAR-Lite via Arduino without
      worrying about specifics
  2.  By reading through this library, users of any platform will get an
      explanation of how to use the various functions of LIDAR-Lite and see an
      Arduino example along side.

  This libary was written by Austin Meyers (AK5A) with PulsedLight Inc. And was
  likely downloaded from:

  https://github.com/PulsedLight3D/Lidar_v2_Arduino_Library

  Visit http://pulsedlight3d.com for documentation and support requests
  
  Library modified by Morgan Dykshorn For Use on the Beaglebone Black

============================================================================= */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include "Lidar.h"

/* =============================================================================

  This var set in setup and used in the read function. It reads the value of
  register 0x40, used largely for sending debugging requests to PulsedLight

============================================================================= */
bool Lidar::errorReporting = false;

Lidar::Lidar()
{
	int handle = 0;
	bool nack = false;
}

/* =============================================================================

  Begin

  Starts the sensor and I2C

  Process
  ------------------------------------------------------------------------------
  1.  Turn on error reporting, off by default
  2.  Start Wire (i.e. turn on I2C)
  3.  Enable 400kHz I2C, 100kHz by default
  4.  Set configuration for sensor

  Parameters
  ------------------------------------------------------------------------------
  - configuration: set the configuration for the sensor
    - default or 0 = equivelent to writing 0x00 to 0x00, i.e. full reset of
      sensor, if you write nothing for configuration or 0, the sensor will init-
      iate normally
    - 1 = high speed setting, set the aquisition count to 1/3 the default (works
      great for stronger singles) can be a little noisier
  - fasti2c: if true i2c frequency is 400kHz, default is 100kHz
  - showErrorReporting: if true reads with errors will print the value of 0x40,
    used primarily for debugging purposes by PulsedLight
  - LidarI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

============================================================================= */
void Lidar::begin(int configuration, bool showErrorReporting){
  errorReporting = showErrorReporting;
  Lidar::handle = i2c_open(1); //  Start I2C

  configure(configuration);
}
/* =============================================================================

  Configure

  Sets the configuration for the sensor, typically this is done in the begin()
  command, but sometimes (especially for multi-sensor applications) you will
  need to do this separately.

  Parameters
  ------------------------------------------------------------------------------
  - configuration: set the configuration for the sensor
    - default or 0 = equivelent to writing 0x00 to 0x00, i.e. full reset of
      sensor, if you write nothing for configuration or 0, the sensor will init-
      iate normally
    - 1 = high speed setting, set the aquisition count to 1/3 the default (works
      great for stronger singles) can be a little noisier
============================================================================= */
void Lidar::configure(int configuration){
  switch (configuration){
    case 0: //  Default configuration
      write(0x00,0x00);
    break;
    case 1: //  Set aquisition count to 1/3 default value, faster reads, slightly
            //  noisier values
      write(0x04,0x00);
    break;
    case 2: //  Low noise, low sensitivity: Pulls decision criteria higher
            //  above the noise, allows fewer false detections, reduces
            //  sensitivity
      write(0x1c,0x20);
    break;
    case 3: //  High noise, high sensitivity: Pulls decision criteria into the
            //  noise, allows more false detections, increses sensitivity
      write(0x1c,0x60);
    break;
  }
}

/* =============================================================================

  Distance

  Read the distance from LIDAR-Lite

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the FirstValueFrom0x8f << 8 and add to SecondValueFrom0x8f This new
      value is the distance.

  Parameters
  ------------------------------------------------------------------------------
  - stablizePreampFlag (optional): Default: true, take aquisition with DC
    stabilization/correction. If set to false, it will read
  - faster, but you will need to sabilize DC every once in awhile (ex. 1 out of
    every 100 readings is typically good).
  - LidarI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

  Example Arduino Usage
  ------------------------------------------------------------------------------
  1.  // take a reading with DC stabilization and the 0x62 default i2c address
      // the distance variable will hold the distance
      int distance = 0
      distance = myLidarInstance.distance();

  2.  // take a reading without DC stabilization and the 0x62 default i2c address
      int distance = 0
      distance = myLidarInstance.distance(false);

  3.  // take a reading with DC stabilization and a custom i2c address of 0x66
      int distance = 0
      distance = myLidarInstance.distance(true,0x66);

  Notes
  ------------------------------------------------------------------------------
    Autoincrement: A note about 0x8f vs 0x0f

    Set the highest bit of any register to "1" if you set the high byte of a
    register and then take succesive readings from that register, then LIDAR-
    Lite automatically increments the register one for each read. An example: If
    we want to read the high and low bytes for the distance, we could take two
    single readings from 0x0f and 0x10, or we could take 2 byte read from reg-
    ister 0x8f. 0x8f = 10001111 and 0x0f = 00001111, meaning that 0x8f is 0x0f
    with the high byte set to "1", ergo it autoincrements.

============================================================================= */
int Lidar::distance(bool stablizePreampFlag, bool takeReference)
{
  if(stablizePreampFlag)
  {
    // Take acquisition & correlation processing with DC correction
    write(0x00,0x04);
  }
  else
  {
    // Take acquisition & correlation processing without DC correction
    write(0x00,0x03);
  }
  // Array to store high and low bytes of distance
  uint8_t distanceArray[2];
  // Read two bytes from register 0x8f. (See autoincrement note above)
  read(0x8f,2,distanceArray,true);
  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  //if there is a failure in sensor communication aka "nack", return the maximum sensor distance
  if (!Lidar::nack)
  {
	return(distance);
  }
  else
  {
	  return(4000);
  }
}

/* =============================================================================
  Signal Strength

  The sensor transmits a focused infrared beam that reflects off of a target,
  with a portion of that reflected signal returning to the receiver. Distance
  can be calculated by taking the difference between the moment of signal trans-
  mission to the moment of signal reception. But successfully receiving a ref-
  lected signal is heavily influenced by several factors. These factors include:
  target distance, target size, aspect, reflectivity

  The relationship of distance (D) to returned signal strength is an inverse
  square. So, with increase in distance, returned signal strength decreases by
  1/D^2 or the square root of the distance.

  Additionally, the relationship of a target's Cross Section (C) to returned
  signal strength is an inverse power of 4.  The LIDAR-Lite sensor transmits a
  focused near-infrared laser beam that spreads at a rate of approximately .5ยบ
  as distance increases. Up to 1 meter it is about the size of the lens. Beyond
  1 meter, approximate beam spread in degrees can be estimated by dividing the
  distance by 100, or ~8 milliradians. When the beam overfills (is larger than)
  the target, the signal returned decreases by 1/C^4 or the fourth root of the
  target's cross section.

  The aspect of the target, or its orientation to the sensor, affects the obser-
  vable cross section and, therefore, the amount of returned signal decreases as
  the aspect of the target varies from the normal.

  Reflectivity characteristics of the target's surface also affect the amount of
  returned signal. In this case, we concern ourselves with reflectivity of near
  infrared wavelengths.

  Process
  ------------------------------------------------------------------------------
  1.  Read one byte from 0x0e

  Parameters
  ------------------------------------------------------------------------------
  - LidarI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.

  Example Usage
  ------------------------------------------------------------------------------
  1.  //  Basic usage with default i2c address, the signalStrength variable will
      //  hold the signalStrength measurement
      int signalStrength = 0;
      signalStrength = myLidarInstance.signalStrength();

  =========================================================================== */
int Lidar::signalStrength()
{
  //  Array to store read value
  uint8_t signalStrengthArray[1];
  //  Read one byte from 0x0e
  read(0x0e, 1, signalStrengthArray, false);
  return((int)((unsigned char)signalStrengthArray[0]));
}

//These functions need changes implemented to work

  /* =============================================================================
    =========================================================================== */
  void Lidar::write(char myAddress, char myValue){
	uint16_t init_sequence1[] = {0xc4, (int)myAddress, (int)myValue};
    //sends sequence over i2c
    int nackCatcher = i2c_send_sequence(Lidar::handle, init_sequence1, 3, 0);
	//write failed catcher
    if(nackCatcher != 1){Lidar::nack = true;}
	//sleeps for 1 ms
    usleep(1000);
  }

/* =============================================================================
  =========================================================================== */
void Lidar::read(char myAddress, int numOfBytes, uint8_t arrayToSave[2], bool monitorBusyFlag){
  int busyFlag = 0;
  if(monitorBusyFlag)
  {
    busyFlag = 1;
  }
  int busyCounter = 0;
  //run while busy
  while(busyFlag != 0)
  {
	uint16_t busy_sequence[] = {0xc4, 0x01, I2C_RESTART, 0xc5, I2C_READ};
	uint8_t busyCheck;

    int nackCatcher = i2c_send_sequence(Lidar::handle, busy_sequence, 5, &busyCheck);
	//checks for nack
    if(nackCatcher != 1){Lidar::nack = true;}

	//ands busyCheck with 00000001 to only use the last bit
	busyFlag = 0x01 & busyCheck;

    busyCounter++;
    if(busyCounter > 9999)
	{
      goto bailout;
    }
  }
  //runs when not busy
  if(busyFlag == 0)
  {
	uint16_t read_sequence1[] = {0xc4, (int)myAddress, I2C_RESTART, 0xc5, I2C_READ};
	uint16_t read_sequence2[] = {0xc4, 0x0f, I2C_RESTART, 0xc5, I2C_READ, I2C_READ};
	uint16_t twoBytesSave;

	//reads either 1 or 2 bytes and saves to arrayToSave(need to fix 2 byte arrayToSave)
	if (numOfBytes == 1)
	{
	    int nackCatcher = i2c_send_sequence(Lidar::handle, read_sequence1, 5, &arrayToSave[0]);
		//detects failed write
		if(nackCatcher != 1){Lidar::nack = true;}
	}
	else
	{
	    int nackCatcher = i2c_send_sequence(Lidar::handle, read_sequence2, 6, (uint8_t)&twoBytesSave);
		//detects failed write
		if(nackCatcher != 1){Lidar::nack = true;}
		//separates the 2 Bytes
		arrayToSave[0] = twoBytesSave & 0xFF;
		arrayToSave[1] = twoBytesSave >> 8;
	}
  }
  //if busy for a long time bails
  if(busyCounter > 9999)
  {
    bailout:
      busyCounter = 0;
  }
}