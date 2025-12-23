#ifndef BUTTON_TEST_HPP
#define BUTTON_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Critical points for force vs displacement curve
const float criticalPoints[3] = {2, 1.8, 1.5}; // in radians
// Steepness for linear parts of force displacement curve
const float steepness[3] = {20, 10, 20}; // Nm/rad

// HydraFOC motor object
HydraFOCMotor motor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5], I2C1_SDA, focCurrentPins[0][0], focCurrentPins[0][1]);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    // Configure I2C
    Wire.begin(I2C0_SDA, I2C0_SCL);

    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset
    pinMode(focCurrentPins[0][0], INPUT);
    pinMode(focCurrentPins[0][1], INPUT);

    // Initialize HydraFOC motor
    motor.begin(Direction::CW, 4.69f, true);
    motor.resetEncoder();

    delay(1000); // Wait for motor to stabilize
    Serial.println("FOC Motor Test Initialized.");

    // Set initial target position
    motor.setTorque(2.f);
}

void loop() {
    // Run FOC control loop
    motor.update();

    // Prints encoder angle
    Serial.print("Encoder Angle (rad): ");
    Serial.println(motor.getPosition());
}

#endif // BUTTON_TEST_HPP