#ifndef HYDRA_FOC_MOTOR_H
#define HYDRA_FOC_MOTOR_H

#include <Arduino.h>
#include <SimpleFOC.h>

// HydraFOCMotor: Wrapper for SimpleFOC motor and driver
class HydraFOCMotor {
public:
    // Construct with motor driver pins and encoder i2c port
    HydraFOCMotor(uint8_t pwmA, uint8_t pwmB, uint8_t pwmC, uint8_t enA, uint8_t enB, uint8_t enC, uint8_t dirPin, uint8_t current0, uint8_t current1);

    // Initialize the motor and driver
    void begin(Direction encDir, float encOffset, bool skipAlign);

    // Resets the encoder
    void resetEncoder();

    // Set target velocity (rad/s)
    void setVelocity(float velocity);

    // Set target position (rad)
    void setPosition(float position);

    // Sets target torque (Nm)
    void setTorque(float torque);

    // Update FOC loop (call in loop)
    void update();

    // Motor monitoring
    void monitor();

    // Get current position (rad)
    float getPosition();

    // Get current velocity (rad/s)
    float getVelocity();

private:
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MagneticSensorI2C encoder;
    uint8_t encoderDirPin;
    LowsideCurrentSense currentSense;
    float targetVelocity;
    float targetPosition;
    float targetTorque;
    enum ControlMode { VELOCITY, POSITION, TORQUE } mode;
};

#endif