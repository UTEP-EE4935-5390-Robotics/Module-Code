/*
 * Module 2 -- Move it!
 */ 

//TODO, Add your group information to the top of your code.

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

// TODO, Section, 4.2: Add line to include Chassis.h

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// TODO, Section 4.2: Declare the chassis object (with default values)
// TODO, Section 6.2: Adjust parameters to better match actual motion

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  // TODO, Section 4.2: Uncomment call to chassis.idle() to stop the motors
  // chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("/idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // TODO, Section 4.2: Initialize the chassis (which also initializes the motors)

  // TODO, Section 5.1: Adjust the PID coefficients

  idle();

  // Initializes the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section 6.1 (but not before!): Edit the function definition to accept a distance and speed
void drive(void)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: In Section 4.2 and 5.1, add a call to chassis.setWheelSpeeds() to set the wheel speeds

  // TODO: In Section 6.1, remove the call to setWheelSpeeds() and add a call to chassis.driveFor()

}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO, Section 6.1: Make a call to chassis.turnFor()

}

// TODO, Section 6.1: Declare function handleMotionComplete(), which calls idle()

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  // TODO, Section 3.2: add "emergency stop"

  switch(robotState)
  {
    case ROBOT_IDLE:
      // TODO, Section 3.2: Handle up arrow button


      // TODO, Section 6.1: Handle remaining arrows
      break;
      
    default:
      break;
  }
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  // TODO, Section 3.1: Temporarily edit to pass true to getKeyCode()
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 

      // TODO, Section 6.1: Uncomment to handle completed motion
      // if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;

    default:
      break;
  }
}
