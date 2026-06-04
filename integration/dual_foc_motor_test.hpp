#ifndef DUAL_FOC_MOTOR_TEST_HPP
#define DUAL_FOC_MOTOR_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"
    
// Loop counter
unsigned long loopCounter = 0;

// HydraFOC motor objects
HydraFOCMotor* motors = new HydraFOCMotor[NUM_FOC_MOTORS] {
    HydraFOCMotor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5], I2C1_SDA, focCurrentPins[0][0], focCurrentPins[0][1]),
    HydraFOCMotor(focMotorPins[1][0], focMotorPins[1][1], focMotorPins[1][2], focMotorPins[1][3], focMotorPins[1][4], focMotorPins[1][5], I2C1_SDA, focCurrentPins[1][0], focCurrentPins[1][1])
};

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    // Configure I2C ports
    Wire.begin(I2C0_SDA, I2C0_SCL);
    Wire1.begin(I2C1_SDA, I2C1_SCL);

    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset

    // Initialize HydraFOC motors
    for (int i = 0; i < NUM_FOC_MOTORS; i++) {
        motors[i].begin(motorDirs[i], encoderElectricAngles[i], true, (i == 0) ? &Wire : &Wire1);
        motors[i].resetEncoder();
    }

    delay(1000); // Wait for motors to stabilize
    Serial.println("Dual FOC Motor Test Initialized.");

    // Set target torques
    motors[0].setTorque(0.5f);
    motors[1].setTorque(0.5f);
}

void loop() {
    // Run FOC control loop
    for (int i = 0; i < NUM_FOC_MOTORS; i++) {
        motors[i].update();
    }

    // Motor variable monitoring
    //motor.monitor();

    // Prints current sensing readings
    /*Serial.print("Current A: ");
    Serial.print(analogRead(focCurrentPins[0][0]));
    Serial.print(" | Current B: ");
    Serial.println(analogRead(focCurrentPins[0][1]));*/

    // Prints encoder angle with full precision (every 100 loop counts)
    if (loopCounter % 100 == 0) {
        for (int i = 0; i < NUM_FOC_MOTORS; i++) {
            Serial.print("Motor ");            Serial.print(i);
            Serial.print(" Position: ");
            Serial.println(motors[i].getPosition(), 6);  // 6 decimal places
        }
    }

    loopCounter++;
}

#endif // DUAL_FOC_MOTOR_TEST_HPP