//Lidar class
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//needed when mixing c and c++ libraries
#ifdef __cplusplus
    extern "C" {
#endif
        #include "lsquaredc.h"

#ifdef __cplusplus
    }
#endif


#ifndef _LIDARLite_H_
#define _LIDARLite_H_

class Lidar
{
  public:
      Lidar();
      void begin(int = 0, bool = false);
      void configure(int = 0);
      int distance(bool = true, bool = true);
      int signalStrength();
      void write(char, char);
      void read(char, int, byte*, bool);
  private:
      static bool errorReporting;
	  int handle;
	  bool nack;
};

#endif