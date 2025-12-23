#include "HydraFOCMotor.h"

HydraFOCMotor::HydraFOCMotor(uint8_t pwmA, uint8_t pwmB, uint8_t pwmC, uint8_t enA, uint8_t enB, uint8_t enC, uint8_t dirPin)
    : motor(11), // 7 pole pairs as example, adjust as needed
      driver(pwmA, pwmB, pwmC, enA, enB, enC),
      encoder(AS5600_I2C),
      encoderDirPin(dirPin),
      targetVelocity(0),
      targetPosition(0),
      targetTorque(0),
      mode(VELOCITY)
{
}

void HydraFOCMotor::begin() {
    SimpleFOCDebug::enable(&Serial);

    digitalWrite(encoderDirPin, HIGH); // Set direction pin high (adjust as needed)

    // initialise magnetic sensor hardware
    encoder.init();
    // link the motor to the sensor
    motor.linkSensor(&encoder);

    // pwm frequency to be used [Hz]
    driver.pwm_frequency = 30000;
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    // Max DC voltage allowed - default voltage_power_supply
    driver.voltage_limit = 5.6;

    driver.init();
    motor.linkDriver(&driver);

    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // contoller configuration
    // default parameters in defaults.h

    // velocity PI controller parameters
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 20;
    motor.PID_velocity.D = 0;
    // maximal voltage to be set to the motor
    motor.voltage_limit = 5.6f;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01f;

    // angle P controller
    motor.P_angle.P = 20;
    // maximal velocity of the position control
    motor.velocity_limit = 20;
    
    // comment out if not needed
    motor.useMonitoring(Serial);

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();
}

void HydraFOCMotor::setVelocity(float velocity) {
    targetVelocity = velocity;
    mode = VELOCITY;
    motor.controller = MotionControlType::velocity;
}

void HydraFOCMotor::setPosition(float position) {
    targetPosition = position;
    mode = POSITION;
    motor.controller = MotionControlType::angle;
}

void HydraFOCMotor::setTorque(float torque) {
    targetTorque = torque;
    mode = TORQUE;
    motor.controller = MotionControlType::torque;
}

void HydraFOCMotor::update() {
    motor.loopFOC();
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
    return motor.shaft_angle;
}

float HydraFOCMotor::getVelocity() const {
    return motor.shaft_velocity;
}