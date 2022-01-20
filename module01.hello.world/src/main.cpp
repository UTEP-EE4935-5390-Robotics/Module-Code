#include <Arduino.h>
#include <wpi-32u4-lib.h>

// TODO: Add some header information here.  Names, date, etc.    Add comments as you go. Future you will thank you.

// TODO: declare the pin for the LED


// Create a button object for the built-in Button A on the Romi
Romi32U4ButtonA buttonA;

// Define two basic states. For this program, they will correspond to an LED state (on or off).
// "enum" stands for "enumerate". Basically, we define a new variable type called ROBOT_STATE.
// We prepend "ROBOT_" to avoid conflicts with other macros that may be defined elsewhere.
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_ACTIVE};

// Declare a variable, robotState, of our new type, ROBOT_STATE. Initialize it to ROBOT_IDLE.
ROBOT_STATE robotState = ROBOT_IDLE;



void handleButtonPress(void)
{
  // Go through the state machine
  if(robotState == ROBOT_IDLE)
  {
      // TODO: Notify us that we're switching to ACTIVE using the serial print
      // TODO: Count code?

      // TODO: Turn the LED on
      

      // Finally, update the state
      robotState = ROBOT_ACTIVE;
  }

  //note that we use else..if for each additional state, so it doesn't get confused
  else if(robotState == ROBOT_ACTIVE)
  {
      // TODO: Notify us that we're switching to IDLE
      
      // BONUS TODO: Output your team names

      // TODO: Turn the LED off
      

      // Finally, update the state
      robotState = ROBOT_IDLE;
  }
}

/*
 * This is the standard setup function that is called when the Romi is rebooted.
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // TODO: Initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately in platformio.ini
  

  // TODO: Set the LED pin to be an OUTPUT
  
}

/* Here is where all the fun happens. For each state, check for and respond to a button press.
 */ 
void loop()
{
  if(buttonA.getSingleDebouncedPress()) handleButtonPress();
}