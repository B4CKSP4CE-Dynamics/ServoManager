#ifndef Servo_h
  #define ServoManagerComp.h

#include "ServoManager.h"


class Servo
{
public:
  Servo();
  uint8_t attach(int pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes. 
  void detach();
  void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  void writeMicroseconds(int value); // Write pulse width in microseconds 
  int read();                        // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                   // return true if this servo is attached, otherwise false 
private:
  uint8_t pin;                      // number of pin for this servo
  int16_t min;                       // minimum is this value times 4 added to MIN_PULSE_WIDTH    
  int16_t max;                       // maximum is this value times 4 added to MAX_PULSE_WIDTH   
};

#endif