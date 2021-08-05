#include <Arduino.h>
#include <RobotServo.h>

// #include <avr/io.h>


#define MID_PULSE_WIDTH (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) >> 1


static servo_ticks servos[MAX_SERVOS];
uint8_t ServoCount = 0;

struct ServoNode {
  uint8_t servo_index;
  ServoNode* next;
};
#define NEW_SERVO_NODE (struct ServoNode*)malloc(sizeof(struct ServoNode))

// all servos ticks data will be stored in this list
struct ServoNode* servos_order = NULL;

















// servo list handling

uint8_t ServoManager::attach(uint8_t pin){     // TODO: impement attach with pulse argument
  // check if pin is open
  for(int i = 0; i < ServoCount; i++)
    if(servos[i].pin == pin)
      return ATTACHMENT_FAILED;
  

  servo_ticks new_servo = {pin, MID_PULSE_WIDTH};
  servos[ServoCount] = new_servo;

  if(servos_order == NULL){     // create list if empty
    servos_order = NEW_SERVO_NODE;
    servos_order->servo_index = ServoCount;
    servos_order->next = NULL;
  }else {                 // else cycle through list to make it sorted
    struct ServoNode* servo = NEW_SERVO_NODE;  
    servos_order->servo_index = ServoCount;

    struct ServoNode* n = servos_order;
    while(n->next != NULL){
      if( servos[n->servo_index].ticks > servos[servo->servo_index].ticks && 
          servos[n->next->servo_index].ticks <= servos[servo->servo_index].ticks)
        break;
      n = n->next;
    }

    servo->next = n->next;
    n->next = servo;
  }

  ServoCount++;
  return ATTACHMENT_SUCCEED;
}

// void ServoManager::detach(uint8_t pin){
//   struct ServoNode* n = servos;
//   while(n->next != NULL){
//     if(n->servo.pin == pin)
//       break;
//     n = n->next;
//   }

//   // i feel like there's memory leak
//   n->servo = n->next->servo;
//   n->next = n->next->next;

//   bruh
// }

void ServoManager::write(uint8_t pin, unsigned int ticks){
  for(int i = 0; i < ServoCount; i++)
    if(servos[i].pin == pin)
      servos[i].ticks = ticks;
}