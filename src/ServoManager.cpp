#include <Arduino.h>

#include <ServoManager.h>

#include <avr/io.h>
#include <avr/interrupt.h>

typedef struct {
  uint8_t pin;                // number of pin servo is attached to
  volatile uint16_t ticks;    // pulse width in ticks
  uint8_t enabled;            // status of servo 
} servo_data;

bool operator> (const servo_data &servo1, const servo_data &servo2){
  if(servo1.ticks > servo2.ticks)
    return true;
  if(servo1.ticks == servo2.ticks && servo1.pin > servo2.pin)
    return true;
  return false;
}

bool operator>= (const servo_data &servo1, const servo_data &servo2){
  if(servo1.ticks > servo2.ticks)
    return true;
  if(servo1.ticks == servo2.ticks && servo1.pin >= servo2.pin)
    return true;
  return false;
}

bool operator< (const servo_data &servo1, const servo_data &servo2) {
  return servo2 >= servo1;
}

bool operator<= (const servo_data &servo1, const servo_data &servo2) {
  return servo2 > servo1;
}

bool operator== (const servo_data &servo1, const servo_data &servo2) {
  if(servo1.ticks != servo2.ticks) return false;
  if(servo1.pin != servo2.pin) return false;
  return true;
}


#define MID_PULSE_WIDTH (MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) >> 1))

#define DEFAULT_SERVO(_pin) (servo_data) {_pin, MID_PULSE_WIDTH, PIN_DISABLED}
#define INVALID_SERVO DEFAULT_SERVO(INVALID_SERVO_PIN)
static servo_data* pin_to_servo[PINS_NUMBER];     // array pointing to servos for quick access via pin number

static servo_data servos[MAX_SERVOS];          // array with all attached servos
uint8_t attachedServoCount = 0;     

static uint8_t ticks_order[MAX_SERVOS];         // array with order of servo pulses represented by pin number
uint8_t enabledServoCount = 0;
#define ORDER_TO_TICK(_i) pin_to_servo[ticks_order[_i]]->ticks;


uint8_t current_order = 0;
uint8_t buffer_ticks_order[MAX_SERVOS];
uint8_t buffer_update = false;

// bit shift by 3 represents prescaler set to 8
#define US_TO_TICKS(_us) ((clockCyclesPerMicrosecond()*_us) >> 3)
#define TICKS_TO_US(_tk) ((_tk<<3) / clockCyclesPerMicrosecond())
#define TICKS_ERROR 24

SIGNAL (TIMER1_COMPA_vect) {
  if(enabledServoCount == 0){
    TCNT1 = 0;
    OCR1A = US_TO_TICKS(CYCLE_WIDTH);
  } else {
    if(TCNT1 >= US_TO_TICKS(CYCLE_WIDTH)) { 
      TCNT1 = 0; 

      for(int i = 0; i < enabledServoCount; i++)
        digitalWrite( buffer_ticks_order[i], HIGH);

      if(buffer_update)
        for(int i = 0; i < MAX_SERVOS; i++)
          buffer_ticks_order[i] = ticks_order[i];
      buffer_update = false;

      OCR1A = pin_to_servo[buffer_ticks_order[current_order]]->ticks;
    } else {
      while(current_order < enabledServoCount){
        digitalWrite(buffer_ticks_order[current_order], LOW);
        current_order++;

        if(current_order >= enabledServoCount){
          current_order = 0;
          OCR1A = US_TO_TICKS(CYCLE_WIDTH);
          break;
        } else {
          OCR1A = pin_to_servo[buffer_ticks_order[current_order]]->ticks;
          if(pin_to_servo[buffer_ticks_order[current_order - 1]]->ticks < OCR1A+TICKS_ERROR)
            break;
        }
      }
    }
  }
}

void initTimer(){
  TCCR1A = 0b00000000;      // normal operating mode
  TCCR1B = 0b00000010;      // prescaler set to clk/8
  TIFR1 |= _BV(OCF1A);      // clear any pending interrupts
  TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare A interrupt
  TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare A interrupt
  TCNT1 = 0;                // clear the timer count
}

// Servo Manager functionality
ServoManager::ServoManager(){
  // init servo array with invalid and pin_to_servo with NULLs
  int i = 0;   
  for(; i < MAX_SERVOS; i++){
    servos[i] = INVALID_SERVO;
    ticks_order[i] = INVALID_SERVO_PIN;
  } 

  for (i = 0; i < PINS_NUMBER; i++){
    pin_to_servo[i] = nullptr;
  }

  initTimer();
}

uint8_t ServoManager::attach(uint8_t pin){     

  // check if not limited by servo number
  if(attachedServoCount == MAX_SERVOS)
    return ATTACHMENT_SERVO_LIMIT;

  // check if pin is open
  if(pin_to_servo[pin] != nullptr)
    return ATTACHMENT_PIN_POPULATED;
  
  // looking for empty spot in servo array
  for(int i = 0; i < MAX_SERVOS; i++){
    if(servos[i].pin == INVALID_SERVO_PIN){
      servos[i] = DEFAULT_SERVO(pin);
      pin_to_servo[pin] = servos + i;
      break;
    }
  }

  attachedServoCount++;
  return ATTACHMENT_SUCCEED;
}

uint8_t ServoManager::detach(uint8_t pin){
  if(pinAttached(pin) == SERVO_FOUND){
    if(pinEnabled(pin) == PIN_ENABLED)
      disable(pin);
    *(pin_to_servo[pin]) = INVALID_SERVO;
    pin_to_servo[pin] = nullptr;

    buffer_update = true;
    return SERVO_FOUND;
  } else 
    return SERVO_NOT_FOUND;
}


uint8_t ServoManager::enable(uint8_t pin){
  if(!pinAttached(pin))
    return SERVO_NOT_FOUND;
  if(pinEnabled(pin))
    return SERVO_FOUND;

  
  // shifting all servos with greater ticks value to the left
  int i;
  for(i = enabledServoCount - 1; i >= 0; i--){
    if(*pin_to_servo[ticks_order[i]] > *pin_to_servo[pin]) 
      ticks_order[i + 1] = ticks_order[i];
    else
      // if found servo with identical or lower ticks value, breaking and inserting here
      break;
  }

  ticks_order[i + 1] = pin;
  pin_to_servo[pin]->enabled = PIN_ENABLED; 

  enabledServoCount += 1; 
  buffer_update = true;
  return SERVO_FOUND;
}

uint8_t ServoManager::write(uint8_t pin, uint16_t ticks){
  if(pinAttached(pin) == SERVO_NOT_FOUND)
    return SERVO_NOT_FOUND;

  // writing servo ticks to servo array
  uint16_t ticks_to_write;
  if(ticks > MAX_PULSE_WIDTH)
    ticks_to_write = MAX_PULSE_WIDTH;
  else if(ticks < MIN_PULSE_WIDTH)
    ticks_to_write = MIN_PULSE_WIDTH;
  else 
    ticks_to_write = ticks;
  
  ticks_to_write = US_TO_TICKS(ticks_to_write);
  servo_data old_servo_data = *pin_to_servo[pin];
  servo_data new_servo_data = *pin_to_servo[pin];
  new_servo_data.ticks = ticks_to_write;

  if(pinEnabled(pin) == PIN_ENABLED){      // adjusting position in servos order if servo is enabled
    if(old_servo_data.ticks < ticks_to_write){
      int i = 0;

      // skipping until we find previous value
      for(; i < enabledServoCount; i++)
        if(*pin_to_servo[ticks_order[i]] == old_servo_data){
          i++;
          break;
        }
      // shifting every value to the left until we find new value
      for(; i < enabledServoCount; i++){
        ticks_order[i - 1] = ticks_order[i];
        if(*pin_to_servo[ticks_order[i]] < new_servo_data){
          i++;
          break;
        }
      }
      // placing pin number in order
      ticks_order[i - 1] = pin;
    } else if(old_servo_data.ticks > ticks_to_write){
      // same, but reversed
      int i = enabledServoCount - 1;

      for(; i >= 0; i--)
        if(*pin_to_servo[ticks_order[i]] == old_servo_data){
          i--;
          break;
        }
      for(; i >= 0; i--){
        ticks_order[i + 1] = ticks_order[i];  
        if(*pin_to_servo[ticks_order[i]] > new_servo_data){
          i--;
          break;
        }
      }
      ticks_order[i + 1] = pin;
    } else 
      return SERVO_FOUND;
  }

  pin_to_servo[pin]->ticks = ticks_to_write;

  buffer_update = true;
  return SERVO_FOUND;
}

uint8_t ServoManager::disable(uint8_t pin){
  if(!pinAttached(pin))
    return SERVO_NOT_FOUND;
  if(!pinEnabled(pin))
    return SERVO_FOUND;

  
  // looking for pin
  int i;
  for(i = 0; i < enabledServoCount; i++)
    if(ticks_order[i] == pin)
      break;

  // shifting rest of servos to the right
  for(; i < enabledServoCount - 1; i++)
    ticks_order[i] = ticks_order[i + 1];

  // plugging left over
  ticks_order[i] = INVALID_SERVO_PIN;


  pin_to_servo[pin]->enabled = PIN_DISABLED;  

  enabledServoCount--;
  buffer_update = true;
  return SERVO_FOUND;
}


uint8_t ServoManager::getServoCount(){
  return attachedServoCount;
}

uint8_t ServoManager::pinAttached(uint8_t pin){
  if(pin >= PINS_NUMBER)
    return SERVO_NOT_FOUND;
  if(pin_to_servo[pin] == nullptr)
    return SERVO_NOT_FOUND;
  return SERVO_FOUND;
}

uint8_t ServoManager::pinEnabled(uint8_t pin){
  if(pinAttached(pin))
    return pin_to_servo[pin]->enabled;
  return PIN_DISABLED;
}