#ifndef ServoManager_h
  #define ServoManager_h


#include <inttypes.h>

// in microseconds
#define MIN_PULSE_WIDTH  1000
#define MAX_PULSE_WIDTH  2000
#define CYCLE_WIDTH     20000

#define MAX_SERVOS            12
#define PINS_NUMBER           14
#define INVALID_SERVO_PIN   0xFF

#define ATTACHMENT_SERVO_LIMIT      0
#define ATTACHMENT_PIN_POPULATED    1
#define ATTACHMENT_SUCCEED          2
#define ATTACHMENT_INVALID_MIN_MAX  3

#define SERVO_NOT_FOUND 0
#define SERVO_FOUND     1

#define PIN_DISABLED    0
#define PIN_ENABLED     1


class ServoManager{
public:

  // slow functions - executed when setting up
  ServoManager();
  uint8_t attach(uint8_t pin);                                                      // attach servo to the given pin
  uint8_t attach(uint8_t pin, uint16_t min_pulse_width, uint16_t max_pulse_width);  // attach servo to the given pin and set min/ax pulse width
  uint8_t detach(uint8_t pin);                                                      // detach servo from its pin, making channel free for new servos

  // fast functions - executed in code
  uint8_t enable(uint8_t pin);                                                      // begin writing PWM signal to servo
  uint8_t write(uint8_t pin, uint16_t ticks);                                       // angle represented with pulse width from bottomed out position 
  uint8_t write_angle(uint8_t pin, uint8_t angle);                                  // angle from bottomed out position 
  uint8_t disable(uint8_t pin);                                                     // pause writing PWM signal to servo

  // return public variables
  uint8_t getServoCount();                                                          // returns number of servos attached 
  uint8_t pinAttached(uint8_t pin);                                                 // returns whether there is servo attached or not to pin
  uint8_t pinEnabled(uint8_t pin);                                                  // returns whether servo on this pin is enabled or not
};

#endif