//Lidar class
#include <lsquaredc.h>

#ifndef _LIDARLite_H_
#define _LIDARLite_H_

class Lidar
{
  public:
      Lidar();
      void begin(int = 0, bool = false, bool = false, char = 0x62);
      void configure(int = 0, char = 0x62);
      void fast(char = 0x62);
      int distance(bool = true, bool = true, char = 0x62);
      int signalStrength(char = 0x62);
      void write(char, char, char = 0x62);
      void read(char, int, byte*, bool, char);
  private:
      static bool errorReporting;     
};

#endif