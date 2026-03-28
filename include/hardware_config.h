/*
Hardware configuration for Raspberry Pi Pico prototype board for Touchpoint Mouse v0.2.
*/

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

//External imports
#include <Arduino.h>
#include <ArduinoEigen.h>

//////////////////////////////////////////////////////////////////
// RTOS configuration
//////////////////////////////////////////////////////////////////

// RTOS core affinity
uint8_t CORE_0 = 0;
uint8_t CORE_1 = 1;


//////////////////////////////////////////////////////////////////
// Communications configuration
//////////////////////////////////////////////////////////////////

#define SERIAL_BAUD 460800
constexpr uint32_t DESKTOP_PACKET_TIMEOUT_MS = 2500;

// Header enum
enum Headers {
    PING = 0xFF,
    ELEVATION = 0x10,
    ELEVATION_SPEED = 0x11,
    VIBRATION_EFFECT = 0x20,
    VIBRATION_INTENSITY = 0x21,
    PIXELS_PER_MM = 0x30
};


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
#define SERVO_PWM_FREQ 200
constexpr uint8_t ZOOM_LOWER_BOUNDARY_EFFECT = 27;
constexpr uint8_t ZOOM_UPPER_BOUNDARY_EFFECT = 27;
constexpr uint8_t CLICK_DOWN_EFFECT = 24;
constexpr uint8_t CLICK_UP_EFFECT = 25;

//////////////////////////////////////////////////////////////////
// Mouse driver configuration
//////////////////////////////////////////////////////////////////

constexpr uint16_t MOUSE_CPI = 1100;
constexpr bool MOUSE_HEADLESS_MODE_ENABLED = true;
const Eigen::Vector2f MOUSE_POINTER_OFFSET_MM(-29.676f, -45.375f);
constexpr float MOUSE_POINTER_SENSITIVITY = 2.0f;
constexpr uint8_t MOUSE_ZOOM_RESOLUTION = 2;
constexpr float MOUSE_ZOOM_RANGE = 0.5f;
constexpr int MOUSE_OPTICAL_ROTATION_DEG = 270;
constexpr bool MOUSE_SCROLL_DIR = true;
constexpr bool MOUSE_ZOOM_DIR = true;

#endif // HARDWARE_CONFIG_H