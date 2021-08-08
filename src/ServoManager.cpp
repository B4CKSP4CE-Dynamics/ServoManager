#include <Arduino.h>
#include <ServoManager.h>

#include <avr/io.h>


#define MID_PULSE_WIDTH (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) >> 1

#define DEFAULT_SERVO(_pin) (servo_ticks) {_pin, MID_PULSE_WIDTH, PIN_DISABLED}
#define INVALID_SERVO DEFAULT_SERVO(INVALID_SERVO_PIN)

static servo_ticks* pin_to_servo[MAX_PINS];     // array pointing to servos for quick access via pin number

static servo_ticks servos[MAX_SERVOS];          // array with all attached servos
uint8_t attachedServoCount = 0;     

static uint8_t ticks_order[MAX_SERVOS];         // array with order of servo pulses represented by pin number
uint8_t enabledServoCount = 0;
#define ORDER_TO_TICK(_i) pin_to_servo[ticks_order[_i]]->ticks;

// Servo Manager functionality
ServoManager::ServoManager(){
  // init servo array with invalid and pin_to_servo with NULLs
  int i = 0;   
  for(i; i < MAX_SERVOS; i++){
    servos[i] = INVALID_SERVO;
    pin_to_servo[i] = NULL;
  } for (i; i < MAX_PINS; i++){
    pin_to_servo[i] = NULL;
  }
}

// TODO: impement attach with pulse argument
uint8_t ServoManager::attach(uint8_t pin){     

  // check if not limited by servo number
  if(attachedServoCount = MAX_SERVOS)
    return ATTACHMENT_FAILED;
  // check if pin is open
  if(pin_to_servo[pin] != NULL)
    return ATTACHMENT_FAILED;
  
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
  if(pinAttached){
    if(pinEnabled)
      disable(pin);
    *(pin_to_servo[pin]) = INVALID_SERVO;
    pin_to_servo[pin] = NULL;
    return SERVO_FOUND;
  }else 
    return SERVO_NOT_FOUND;
}


uint8_t ServoManager::enable(uint8_t pin){
  if(!pinAttached(pin))
    return SERVO_NOT_FOUND;
  if(pinEnabled(pin))
    return SERVO_FOUND;

  
  // shifting all servos with greater ticks vlue to the left
  int i;
  for(i = enabledServoCount - 1; i >= 0; i--){
    if(pin_to_servo[ticks_order[i]]->ticks > pin_to_servo[pin]->ticks)      // TODO: potential optimization
      ticks_order[i + 1] = ticks_order[i];
    else
      // if found servo with identical or lower ticks value, breaking and inserting here
      break;
  }

  ticks_order[i + 1] = pin;
  pin_to_servo[pin]->enabled = PIN_ENABLED;  
  return SERVO_FOUND;
}

// TODO: implement buffering
uint8_t ServoManager::write(uint8_t pin, uint16_t ticks){
  if(!pinAttached)
    return SERVO_NOT_FOUND;

  // writing servo ticks to servo array
  uint16_t ticks_to_write;
  if(ticks > MAX_PULSE_WIDTH)
    ticks_to_write = MAX_PULSE_WIDTH;
  else if(ticks < MIN_PULSE_WIDTH)
    ticks_to_write = MIN_PULSE_WIDTH;
  else 
    ticks_to_write = ticks;
  
  uint16_t prev_ticks = pin_to_servo[pin]->ticks;
  pin_to_servo[pin]->ticks = ticks_to_write;

  if(pinEnabled(pin))      // adjusting position in servos order if servo is enabled
    if(prev_ticks < ticks_to_write){
      int i = 0;
      for(i; pin_to_servo[ticks_order[i]]->ticks <= prev_ticks; i++)            // looking for previous ticks value
        ;
      for(i--; pin_to_servo[ticks_order[i]]->ticks <= ticks_to_write; i++)   // shifting positions to the left 
        ticks_order[i] = ticks_order[i + 1];
      ticks_order[i] = ticks_to_write;
    } else if(prev_ticks > ticks_to_write){
      int i = enabledServoCount - 1;
      for(i; pin_to_servo[ticks_order[i]]->ticks <= prev_ticks; i--)
        ;
      for(i++; pin_to_servo[ticks_order[i]]->ticks <= ticks_to_write; i--)
        ticks_order[i] = ticks_order[i - 1];  
      ticks_order[i] = ticks_to_write;
    }

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
  for(i; i < enabledServoCount - 1; i++)
    ticks_order[i] == ticks_order[i + 1];

  pin_to_servo[pin]->enabled = PIN_DISABLED;  
  return SERVO_FOUND;
}


uint8_t ServoManager::getServoCount(){
  return attachedServoCount;
}

uint8_t ServoManager::pinAttached(uint8_t pin){
  if(pin >= MAX_PINS)
    return SERVO_NOT_FOUND;
  if(pin_to_servo[pin] == NULL)
    return SERVO_NOT_FOUND;
  return SERVO_FOUND;
}

uint8_t ServoManager::pinEnabled(uint8_t pin){
  if(pinAttached(pin))
    return pin_to_servo[pin]->enabled;
  return PIN_DISABLED;
}
