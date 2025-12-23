#include "HydraFOCMotor.h"

HydraFOCMotor::HydraFOCMotor(uint8_t pwmA, uint8_t pwmB, uint8_t pwmC, uint8_t enA, uint8_t enB, uint8_t enC, uint8_t dirPin, uint8_t current0, uint8_t current1)
    : motor(11),
      driver(pwmA, pwmB, pwmC, enA, enB, enC),
      encoder(AS5600_I2C),
      encoderDirPin(dirPin),
      currentSense(0.025f, 100.f, current0, current1),
      targetVelocity(0),
      targetPosition(0),
      targetTorque(0),
      mode(VELOCITY)
{
}

void HydraFOCMotor::begin(Direction encDir, float encOffset, bool skipAlign) {
    //SimpleFOCDebug::enable(&Serial);

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

    // default parameters in defaults.h

    // velocity PI controller parameters
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 16.f;
    motor.PID_velocity.D = 0;
    // maximal voltage to be set to the motor
    motor.voltage_limit = 5.6f;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01f;

    // angle P controller
    motor.P_angle.P = 20;
    // maximal velocity of the controller
    motor.velocity_limit = 150;
    
    // comment out if not needed
    //motor.useMonitoring(Serial);
    // Set monitoring variables to include q and d currents, along with velocity and angle
    //motor.monitor_variables = _MON_CURR_Q;
    //motor.monitor_downsample = 100; // default 10

    // link current sense to driver and motor BEFORE motor.init()
    currentSense.linkDriver(&driver);

    // initialize motor
    motor.init();
    
    // initialize current sense AFTER motor.init()
    currentSense.init();
    motor.linkCurrentSense(&currentSense);
    
    //Calibration parameters
    motor.zero_electric_angle = encOffset;
    motor.sensor_direction = encDir;
    currentSense.skip_align = skipAlign; // skip current sense alignment

    // align sensor and start FOC
    motor.initFOC();
}

void HydraFOCMotor::resetEncoder() {
    motor.sensor_offset = encoder.getAngle();
}

void HydraFOCMotor::setVelocity(float velocity) {
    targetVelocity = velocity;
    mode = VELOCITY;
    motor.controller = MotionControlType::velocity;
}

float HydraFOCMotor::getTargetVelocity() {
    return targetVelocity;
}

void HydraFOCMotor::setPosition(float position) {
    targetPosition = position;
    mode = POSITION;
    motor.controller = MotionControlType::angle;
}

float HydraFOCMotor::getTargetPosition() {
    return targetPosition;
}

void HydraFOCMotor::setTorque(float torque) {
    targetTorque = torque;
    mode = TORQUE;
    motor.controller = MotionControlType::torque;
}

float HydraFOCMotor::getTargetTorque() {
    return targetTorque;
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

void HydraFOCMotor::monitor() {
    motor.monitor();
}

float HydraFOCMotor::getPosition() {
    return encoder.getAngle() - motor.sensor_offset;  // Get position directly from encoder
}

float HydraFOCMotor::getVelocity() {
    return encoder.getVelocity();  // Get velocity directly from encoder
}