#ifndef SERVO_TEST_HPP
#define SERVO_TEST_HPP

#include <Arduino.h>
#include <Button.h>
#include "hardware_config.h"
#include "DigitalServo.h"

DigitalServo testServo;
Button testButton(3, PullMode::PULLUP);

uint8_t currentAngle = 0;

void setup() {
    Serial.begin(115200);

    testServo.attach(SERVO_PWM);
    testServo.setPWMFrequency(SERVO_PWM_FREQ);

    pinMode(RIGHT_BUTTON, INPUT_PULLUP);
    testButton.attach(RIGHT_BUTTON);
}

void loop() {
    // Move servo one step every time the button is pressed
    testButton.update();
    if (testButton.changeTo(HIGH)) {
        Serial.print("Button pressed, moving servo to ");
        Serial.print(currentAngle);
        Serial.println(" degrees");
        testServo.writeDegrees(currentAngle);
        currentAngle += 1; // Increment angle by 1 degree
        if (currentAngle > 180) {
            currentAngle = 0; // Wrap around to 0 after reaching 180
        }
    }
    Serial.flush();
}

#endif // SERVO_TEST_HPP