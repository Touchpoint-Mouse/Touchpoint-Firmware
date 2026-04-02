/*
  step_tracker.ino - Tracks rotary encoder steps and direction
  Created by Carson G. Ray, August 12 2022.
  Released into the public domain.
*/

#include <RotEncoder.h>

#define A 3
#define B 4

//Creates rotary encoder instance with click and delay pins
RotEncoder encoder = RotEncoder(EncoderResolution::SINGLE);

void setup() {
  Serial.begin(9600);

  encoder.attach(A, B);
}

void loop() {
  //Updates encoder state
  encoder.update();

  //If encoder has moved a click
  if (encoder.change() != 0) {
    //Logs encoder direction
    Serial.print("Direction: ");
    if (encoder.dir()) {
      Serial.println("Clockwise");
    } else {
      Serial.println("Counterclockwise");
    }
  
    //Logs total clicks of encoder since reset
    Serial.print("Total Steps: ");
    Serial.println(encoder.totalSteps());
  
    //Tracks position of encoder were clockwise is positive
    Serial.print("Net Steps: ");
    Serial.println(encoder.netSteps());
  }
}
