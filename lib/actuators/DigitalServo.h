/*
    * DigitalServo.h - Library for controlling digital servos with precise timing.
    * Created by Carson G. Ray on 2024-06-01.
*/

#ifndef DIGITAL_SERVO_H
#define DIGITAL_SERVO_H

#include <Arduino.h>

class DigitalServo {
    public:
        DigitalServo();
        void attach(uint8_t pin);
        void setPWMFrequency(uint16_t freq);
        void writeMicroseconds(uint16_t us);
        void writeDegrees(float degrees);
        bool attached();

    private:
        uint8_t servoPin;
        bool isAttached;

#ifdef ARDUINO_ARCH_RP2040
        uint pwmSlice;
        uint pwmChannel;
        uint16_t pwmWrap;
#endif

        // Servo parameters

        // PWM frequency
        static const uint16_t pwmFreq;
        // Center of servo range
        static const uint16_t rangeCenter;
        // Length of servo range (one side)
        static const uint16_t rangeLength;
        // Deadband width
        static const uint16_t deadband;
};

#endif // DIGITAL_SERVO_H
        