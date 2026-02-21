/*
    * DigitalServo.cpp - Library for controlling digital servos with precise timing.
    * Created by Carson G. Ray on 2024-06-01.
*/

#include "DigitalServo.h"

#ifdef ARDUINO_ARCH_RP2040
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#endif

// Initialize static constants
const uint16_t DigitalServo::pwmFreq = 200;
const uint16_t DigitalServo::rangeCenter = 1500;
const uint16_t DigitalServo::rangeLength = 750;
const uint16_t DigitalServo::deadband = 6;

DigitalServo::DigitalServo() : servoPin(0), isAttached(false) {
#ifdef ARDUINO_ARCH_RP2040
    pwmSlice = 0;
    pwmChannel = 0;
    pwmWrap = 0;
#endif
}

void DigitalServo::attach(uint8_t pin) {
    servoPin = pin;
    pinMode(servoPin, OUTPUT);
    
#ifdef ARDUINO_ARCH_RP2040
    // Get PWM slice and channel for this pin
    pwmSlice = pwm_gpio_to_slice_num(servoPin);
    pwmChannel = pwm_gpio_to_channel(servoPin);
    
    // Configure GPIO for PWM
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
#endif
    
    setPWMFrequency(pwmFreq);
    isAttached = true;
}

void DigitalServo::setPWMFrequency(uint16_t freq) {
#ifdef ARDUINO_ARCH_RP2040
    // Get system clock frequency (typically 125 MHz)
    uint32_t clockFreq = clock_get_hz(clk_sys);
    
    // Calculate divider and wrap for desired frequency
    // We want 1 MHz clock for 1us resolution (125 MHz / 125 = 1 MHz)
    float divider = (float)clockFreq / 1000000.0f; // 1 MHz
    
    // Wrap value for desired frequency at 1 MHz
    // For 50 Hz: 1,000,000 / 50 = 20,000 counts (20ms period)
    pwmWrap = 1000000 / freq;
    
    // Configure PWM
    pwm_set_clkdiv(pwmSlice, divider);
    pwm_set_wrap(pwmSlice, pwmWrap - 1); // wrap is 0-indexed
    pwm_set_enabled(pwmSlice, true);
#else
    // Placeholder for other platforms (ESP32, etc.)
    // ESP32 uses ledc functions which should be implemented here
#endif
}

void DigitalServo::writeMicroseconds(uint16_t us) {
    if (!isAttached) return;
    
    // Constrain pulse width to valid range
    us = constrain(us, rangeCenter - rangeLength, rangeCenter + rangeLength);
    
#ifdef ARDUINO_ARCH_RP2040
    // With 1 MHz PWM clock, 1 count = 1 microsecond
    // Set PWM level (duty cycle) to achieve desired pulse width
    pwm_set_chan_level(pwmSlice, pwmChannel, us);
#else
    // Placeholder for other platforms (ESP32, etc.)
    // ESP32 should use ledc_set_duty and ledc_update_duty
#endif
}

void DigitalServo::writeDegrees(float degrees) {
    if (!isAttached) return;
    
    // Constrain degrees to valid range (0-180)
    degrees = constrain(degrees, 0.0f, 180.0f);
    
    // Map degrees to pulse width
    // 0° → rangeCenter - rangeLength (900us)
    // 90° → rangeCenter (1500us)
    // 180° → rangeCenter + rangeLength (2100us)
    uint16_t us = rangeCenter - rangeLength + (degrees / 180.0f) * (2 * rangeLength);
    writeMicroseconds(us);
}

bool DigitalServo::attached() {
    return isAttached;
}
