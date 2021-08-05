#ifndef Servo_h
#define Servo_h

#include <inttypes.h>


#define MIN_PULSE_WIDTH 1550
#define MAX_PULSE_WIDTH 2450
#define CYCLE_WIDTH     2000

#define MAX_SERVOS 12

#define ATTACHMENT_FAILED 0
#define ATTACHMENT_SUCCEED 1



typedef struct {
  uint8_t pin;
  volatile unsigned int ticks;
} servo_ticks;

class ServoManager{
public:
  uint8_t attach(uint8_t pin);                  // attach the given pin to the next free channel, sets pinMode
//   void detach(uint8_t pin);                     // detach servo from its pin, making channel free for new servos
  void write(uint8_t pin, unsigned int ticks);  // angle from bottomed out position 
  uint8_t ServoCount;
};

#endif