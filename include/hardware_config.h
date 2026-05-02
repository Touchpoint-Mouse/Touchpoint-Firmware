/*
Hardware configuration for Raspberry Pi Pico prototype board for Touchpoint Mouse v0.2.
*/

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

//External imports
#include <Arduino.h>
#include <ArduinoEigen.h>

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
#define ZOOMWHEEL_SW 15

#define SCROLLWHEEL_A 17
#define SCROLLWHEEL_B 18

#define LEFT_BUTTON 13
#define RIGHT_BUTTON 16

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
constexpr uint8_t MOUSE_ZOOM_RANGE = 3;
constexpr int MOUSE_OPTICAL_ROTATION_DEG = 270;
constexpr bool MOUSE_SCROLL_DIR = true;
constexpr bool MOUSE_ZOOM_DIR = true;

//////////////////////////////////////////////////////////////////
// Servo lookup table
//////////////////////////////////////////////////////////////////

const uint8_t SERVO_ANGLE_START = 45;
const float SERVO_HEIGHTS_MM[] = {
    0.f, 0.08f, 0.22f, 0.39f, 0.55f, 0.69f, 0.81f, 0.99f, 1.13f,
    1.23f, 1.43f, 1.50f, 1.70f, 1.83f, 1.95f, 2.07f, 2.23f, 2.46f, 2.54f, 2.65f,
    2.88f, 3.04f, 3.19f, 3.39f, 3.48f, 3.67f, 3.83f, 3.95f, 4.10f, 4.21f, 4.41f,
    4.57f, 4.64f, 4.80f, 4.92f, 5.13f, 5.30f, 5.49f, 5.61f, 5.85f, 6.05f, 6.24f,
    6.43f, 6.60f, 6.82f, 6.97f, 7.06f, 7.20f, 7.41f, 7.55f, 7.74f, 7.87f, 8.01f,
    8.18f, 8.36f, 8.53f, 8.64f, 8.72f, 8.94f, 9.05f, 9.23f, 9.34f, 9.51f, 9.65f,
    9.86f, 9.95f, 10.15f, 10.35f, 10.50f, 10.62f, 10.70f, 10.93f, 11.05f, 11.35f,
    11.49f, 11.69f, 11.85f, 11.94f, 12.16f, 12.40f, 12.50f, 12.76f, 12.90f, 13.05f,
    13.24f, 13.45f, 13.67f, 13.79f, 14.0f, 14.17f, 14.35f, 14.54f, 14.65f, 14.76f,
    14.97f, 15.19f, 15.39f, 15.62f, 15.72f, 15.92f, 16.03f, 16.23f, 16.36f, 16.56f,
    16.63f, 16.88f, 17.10f, 17.41f, 17.66f, 17.60f, 17.80f, 17.97f, 18.07f, 18.27f,
    18.55f, 18.65f, 19.07f, 19.32f, 19.56f, 19.63f, 19.81f, 19.98f
};

#endif // HARDWARE_CONFIG_H