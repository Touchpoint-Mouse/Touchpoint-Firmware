#ifndef SERVO_TEST_HPP
#define SERVO_TEST_HPP

#include <Arduino.h>
#include "V0_2_Config.h"
#include "DigitalServo.h"

DigitalServo testServo;
void setup() {
    testServo.attach(SERVO_PWM);
}

void loop() {
    // Sweep servo from 0 to 180 degrees and back
    for (int pos = 0; pos <= 180; pos += 1) {
        testServo.writeDegrees(pos);
        delay(15); // Adjust delay for speed of sweep
    }
    for (int pos = 180; pos >= 0; pos -= 1) {
        testServo.writeDegrees(pos);
        delay(15);
    }
}

#endif // SERVO_TEST_HPP