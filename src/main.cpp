#include "Arduino.h"
// #include "Servo.h"
#include "RobotServo.h"

// int brightness = 10;
int led_pin = 13;

ServoManager led_servo;

void setup() {
  led_servo.attach(led_pin);
}


void loop() {
  // led_servo.write(179);
}
