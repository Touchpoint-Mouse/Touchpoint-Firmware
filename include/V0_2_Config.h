/*
Hardware configuration for Raspberry Pi Pico prototype board for Touchpoint Mouse v0.2.
*/

#ifndef V0_2_CONFIG_H
#define V0_2_CONFIG_H

//External imports
#include <Arduino.h>

//////////////////////////////////////////////////////////////////
// RTOS configuration
//////////////////////////////////////////////////////////////////

// RTOS core affinity
uint8_t CORE_0 = 0;
uint8_t CORE_1 = 1;

//////////////////////////////////////////////////////////////////
// Sensor configuration
//////////////////////////////////////////////////////////////////

#define SCL 21
#define SDA 20

#define IMU_MOSI 11
#define IMU_MISO 12
#define IMU_SCK 10
#define IMU_CS 9
#define IMU_INT 6
#define IMU_RST 7

#define OPTICAL_MOSI 3
#define OPTICAL_MISO 0
#define OPTICAL_SCK 2
#define OPTICAL_CS 5
#define OPTICAL_RST 4
#define OPTICAL_INT 1

//////////////////////////////////////////////////////////////////
// User interface configuration
//////////////////////////////////////////////////////////////////

#define ZOOMWHEEL_A 26
#define ZOOMWHEEL_B 27

#define SCROLLWHEEL_A 17
#define SCROLLWHEEL_B 18

#define LEFT_BUTTON 13
#define RIGHT_BUTTON 28

#define NEOPIXEL_DIN 8

//////////////////////////////////////////////////////////////////
// Haptic actuator configuration
//////////////////////////////////////////////////////////////////

#define SERVO_PWM 14

#endif // V0_2_CONFIG_H