#include <Arduino.h>

#include <ServoManagerComp.h>

ServoManager servoManager = ServoManager();

Servo::Servo(){
        
}

uint8_t Servo::attach(int pin){
  this->pin = pin;
  this->min = MIN_PULSE_WIDTH;
  this->max = MAX_PULSE_WIDTH;

  if(servoManager.attach(pin) != ATTACHMENT_SUCCEED)
    return 0;
  
  if(servoManager.enable(pin) != SERVO_FOUND)
    return 0;
}

uint8_t Servo::attach(int pin, int min, int max){
  this->pin = pin;
  this->min = min;
  this->max = max;

  if(servoManager.attach(pin, min, max) != ATTACHMENT_SUCCEED)
    return 0;
  
  if(servoManager.enable(pin) != SERVO_FOUND)
    return 0;
}

void Servo::detach(){
  servoManager.detach(this->pin);
  this->pin = INVALID_SERVO_PIN;
}

void Servo::write(int value){
  servoManager.write_angle(this->pin, value);
}

void Servo::writeMicroseconds(int value){
  servoManager.write(this->pin, value);
}

int Servo::read(){
  return map(readMicroseconds(), this->min, this->max, 0, 180);
}

int Servo::readMicroseconds(){
  return servoManager.pinEnabled(this->pin);
}

bool Servo::attached(){
  return this->pin != INVALID_SERVO_PIN;
}
