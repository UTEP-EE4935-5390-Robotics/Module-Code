/*
 * Simple code to read the line sensors. Used in Activity 2, Staying on Track, Section 4.3.
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  pinMode(LEFT_LINE_SENSE, INPUT);
  pinMode(RIGHT_LINE_SENSE, INPUT);

  Serial.println("/setup()");
}


void loop()
{
  int leftLineSensorReading = analogRead(LEFT_LINE_SENSE);
  int rightLineSensorReading = analogRead(RIGHT_LINE_SENSE);

  Serial.print(leftLineSensorReading);
  Serial.print('\t'); //print a TAB character to make the output prettier
  Serial.println(rightLineSensorReading);

  // In general, delay() is bad because it stalls the program, but this code
  // is just for demonstrating how to read the sensors and we want the output to be legible
  delay(100); 
}
