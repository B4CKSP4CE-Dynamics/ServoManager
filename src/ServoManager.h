#ifndef ServoManager_h
  #define ServoManager_h


#include <inttypes.h>

// in microseconds
#define MIN_PULSE_WIDTH   550
#define MAX_PULSE_WIDTH  2550
#define CYCLE_WIDTH     20000

#define MAX_SERVOS 12
#define MAX_PINS 14
#define INVALID_SERVO_PIN 0xFF

#define ATTACHMENT_SERVO_LIMIT 0
#define ATTACHMENT_PIN_POPULATED 1
#define ATTACHMENT_SUCCEED 2

#define SERVO_NOT_FOUND 0
#define SERVO_FOUND 1

#define PIN_DISABLED 0
#define PIN_ENABLED 1


class ServoManager{
public:

  // slow functions - executed when setting up
  ServoManager();
  uint8_t attach(uint8_t pin);                    // attach the given pin to the next free channel, sets pinMode
  uint8_t detach(uint8_t pin);                    // detach servo from its pin, making channel free for new servos

  // fast functions - executed in code
  uint8_t enable(uint8_t pin);                    // begin writing PWM signal to servo
  uint8_t write(uint8_t pin, uint16_t ticks);     // angle represented with pulse width from bottomed out position 
  uint8_t disable(uint8_t pin);                   // pause writing PWM signal to servo

  // return public variables
  uint8_t getServoCount();                        // returns number of servos attached 
  uint8_t pinAttached(uint8_t pin);               // returns whether there is servo attached or not to pin
  uint8_t pinEnabled(uint8_t pin);                // returns whether servo on this pin is enabled or not

  // debug functions - not expected in release 
  uint8_t printServoOrder();
  uint8_t printServoOrder(String tag);
};

#endif