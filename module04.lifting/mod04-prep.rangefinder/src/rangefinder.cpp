#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <Rangefinder.h>

Rangefinder rangefinder(11, 4);

void setup() 
{
  // This will initialize the Serial as 115200 for prints
  Serial.begin(115200);

  // Call init() to set up the rangefinder
  rangefinder.init();
}

void loop() 
{
  float distance = rangefinder.getDistance();

  // We add a short delay to make the output more legible -- you wouldn't want this in most code
  delay(50);

  Serial.print(millis());
  Serial.print('\t');
  Serial.print(distance); 
  Serial.print(" cm\n");
}
