#ifndef BUTTON_TEST_HPP
#define BUTTON_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Loop counter
unsigned long loopCounter = 0;

// Critical points for force vs displacement curve
const float criticalPoints[3] = {2, 1.5, 1}; // in radians
// Steepness for linear parts of force displacement curve
const float steepness = 50; // Volts/rad
// Constant force
const float constantForce = 1.f; // Volts

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
}

void updateSim() {
    // Get current position
    float position = motor.getPosition();

    // Determine motor control method
    if (position > criticalPoints[1]) {
        motor.setPosition(criticalPoints[0]);
    } else if (position > criticalPoints[2]) {
        motor.setTorque(constantForce);
    } else {
        motor.setTorque(steepness * (criticalPoints[2] - position) + constantForce);
    }
}

void loop() {
    // Run button simulation
    updateSim();

    // Run FOC control loop
    motor.update();

    // Prints position
    if (loopCounter % 100 == 0) {
        Serial.println(motor.getPosition(), 6);
    }

    loopCounter++;
}

#endif // BUTTON_TEST_HPP