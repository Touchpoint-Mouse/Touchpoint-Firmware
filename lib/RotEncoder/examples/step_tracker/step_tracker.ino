/*
  step_tracker.ino - Tracks rotary encoder steps and direction
  Created by Carson G. Ray, August 12 2022.
  Released into the public domain.
*/

#include <RotEncoder.h>

#define A 3
#define B 4

//Creates rotary encoder instance with click and delay pins
RotEncoder encoder = RotEncoder(A, B);

void setup() {
  Serial.begin(9600);
}

void loop() {
  //Updates encoder state
  encoder.update();

  //If encoder has moved a click
  if (encoder.hasMoved()) {
    //Logs encoder direction
    Serial.print("Direction: ");
    if (encoder.dir()) {
      Serial.println("Clockwise");
    } else {
      Serial.println("Counterclockwise");
    }
  
    //Logs total clicks of encoder since reset
    Serial.print("Total Steps: ");
    Serial.println(encoder.steps());
  
    //Tracks position of encoder were clockwise is positive
    Serial.print("Net Steps: ");
    Serial.println(encoder.cw_steps());
  }
}
