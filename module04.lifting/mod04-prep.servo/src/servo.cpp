#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <servo32u4.h>

Servo32U4 servo;

// Declare variables to hold the limits for the servo
#define SERVO_DOWN 1000
#define SERVO_UP 2000

void setup() 
{
  // This will initialize the Serial as 115200 for prints
  Serial.begin(115200);

  // Call attach() to set up the servo
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
}

void loop() 
{
  servo.writeMicroseconds(SERVO_DOWN);
  delay(2000);
  servo.writeMicroseconds(SERVO_UP);
  delay(2000);
}
