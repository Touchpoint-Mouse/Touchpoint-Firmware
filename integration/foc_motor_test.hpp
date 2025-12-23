#ifndef FOC_MOTOR_TEST_HPP
#define FOC_MOTOR_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Motor parameters
constexpr float MOTOR_POLE_PAIRS = 7;
constexpr float MOTOR_KV = 100.0f;

// HydraFOC motor object
HydraFOCMotor motor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5]);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset
    pinMode(focCurrentPins[0][0], INPUT);
    pinMode(focCurrentPins[0][1], INPUT);

    // Initialize HydraFOC motor
    motor.begin();
}

void loop() {
    // Example: Set target velocity
    motor.setVelocity(1.0f); // 10 rad/s

    // Run FOC control loop
    motor.update();

    // Prints current sensing readings
    Serial.print("Current A: ");
    Serial.print(analogRead(focCurrentPins[0][0]));
    Serial.print(" | Current B: ");
    Serial.println(analogRead(focCurrentPins[0][1]));
}

#endif // FOC_MOTOR_TEST_HPP