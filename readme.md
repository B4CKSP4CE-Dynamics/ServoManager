## Description
Simplified library for controlling up to 12 servo motors using singe TCB timer on `ATMEGA328P`. While it doesn't **yet** supports other controllers, it provides a bit more control over multiple servos. 
by @sebekerga

## Genaral use
**1. Initialize ServoManger**
   ```c++
   ServoManger servoManager = ServoManager(); 
   ```
**2. Attach Servos**
   ```c++
   servoManager.attach(pin_number);
   ```
   Note, that this doesn't send any PWM signals to servo yet, it just registers servo and marks given pin as populated.
   
   Alternatively, is you want to adjust Servo's minimum and maximum pulse width from default values:
   ```c++
   servoManager.attach(pin_number, min_pulse_width, min_pulse_width);
   ```
**3. Enable Servo**
   ```c++
   servoManager.enables(pin_number);
   ```
   This function begins sending signal to Servo.

**4. Write Servo pulse value**
   ```c++
   servoManager.write(pin_number, pulse_width_in_us);
   ```
   or 
   ```c++
   servoManager.write_angle(pin_number, angle_from_0_to_180);
   ```
   In `write_angle` minimum and maximum pulse width are used to map angle to pulse width. 

**5. Disable Servo**
   ```c++
   servoManager.disable(pin_number);
   ```
   This function stops sending signal to Servo.

**6. Detach Servo**
   ```c++
   servoManager.detach(pin_number);
   ```
   Disables servo and frees pin, so new servos can be attached on this pin

## Example
```c++
#include "Arduino.h"
#include "ServoManager.h"

#define ANIMATION_DELAY 1000
#define LED_PIN 13
#define SECOND_PIN 8

ServoManager servoManager;

void setup() {
  servoManager = ServoManager();

  servoManager.attach(LED_PIN);
  servoManager.write_angle(LED_PIN, 90);
  servoManager.attach(SECOND_PIN);
  servoManager.write_angle(SECOND_PIN, 90);


  servoManager.enable(LED_PIN);
  servoManager.enable(SECOND_PIN);
}


void loop() {
  delay(ANIMATION_DELAY);
  servoManager.write_angle(LED_PIN, 45);
  servoManager.write_angle(SECOND_PIN, 135);

  delay(ANIMATION_DELAY);
  servoManager.write_angle(LED_PIN, 135);
  servoManager.write_angle(SECOND_PIN, 45);
}
```
>Alternates two servo position in opposite order.

## Arduino compatibility
Alternativly it is possible to include library as `ServoManagerComp`. This way it can be used the same way as Arduino [Servo library](https://www.arduino.cc/reference/en/libraries/servo/), but it is still **yet** limited to `ATMEGA328P`.
