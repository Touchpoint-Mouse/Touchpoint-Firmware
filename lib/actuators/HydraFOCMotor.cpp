#include "HydraFOCMotor.h"

HydraFOCMotor::HydraFOCMotor(uint8_t pwmA, uint8_t pwmB, uint8_t pwmC, uint8_t enA, uint8_t enB, uint8_t enC, uint8_t dirPin, TwoWire* i2cPort)
    : motor(11), // 7 pole pairs as example, adjust as needed
      driver(pwmA, pwmB, pwmC, enA, enB, enC),
      encoder(dirPin, i2cPort),
      targetVelocity(0),
      targetPosition(0),
      targetTorque(0),
      mode(VELOCITY)
{
}

void HydraFOCMotor::begin() {
    // pwm frequency to be used [Hz]
    driver.pwm_frequency = 30000;
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    // Max DC voltage allowed - default voltage_power_supply
    driver.voltage_limit = 5.6;

    driver.init();
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();
    //motor.initFOC();

    // Initialize encoder - read initial position first, then reset to zero
    encoder.begin();
    encoder.reset();   // Now set relative position to zero
}

void HydraFOCMotor::setVelocity(float velocity) {
    targetVelocity = velocity;
    mode = VELOCITY;
    motor.controller = MotionControlType::velocity_openloop;
}

void HydraFOCMotor::setPosition(float position) {
    targetPosition = position;
    mode = POSITION;
    motor.controller = MotionControlType::angle_openloop;
}

void HydraFOCMotor::setTorque(float torque) {
    targetTorque = torque;
    mode = TORQUE;
    motor.controller = MotionControlType::torque;
}

void HydraFOCMotor::update() {
    encoder.update();
    switch (mode) {
        case VELOCITY:
            motor.move(targetVelocity);
            break;
        case POSITION:
            motor.move(targetPosition);
            break;
        case TORQUE:
            motor.move(targetTorque);
            break;
    }
}

float HydraFOCMotor::getPosition() const {
    return encoder.getRelAngle();
}

float HydraFOCMotor::getVelocity() const {
    return motor.shaft_velocity;
}