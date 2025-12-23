#ifndef HYDRA_FOC_CONFIG_H
#define HYDRA_FOC_CONFIG_H

//External imports
#include <Arduino.h>

//////////////////////////////////////////////////////////////////
// RTOS configuration
//////////////////////////////////////////////////////////////////

// RTOS core affinity
uint8_t CORE_0 = 0;
uint8_t CORE_1 = 1;

////////////////////////////////////////////////////////////////////
// I2C and Serial configuration
////////////////////////////////////////////////////////////////////

// I2C pins
#define I2C0_SDA 37
#define I2C0_SCL 38
#define I2C1_SDA 39
#define I2C1_SCL 40

// I2C parameters
#define I2C_BAUD_RATE 1000000
#define I2C_TIMEOUT 1000 // in milliseconds

//Serial parameters
#define SERIAL_BAUD_RATE 115200//460800

////////////////////////////////////////////////////////////////////
// Servo configuration
//////////////////////////////////////////////////////////////////

// Servo signal multiplier for deserialization
const float servoSignalDeserialize = 1./32767.;

////////////////////////////////////////////////////////////////////
// HydraFOC motor configuration
////////////////////////////////////////////////////////////////////

#define NUM_FOC_MOTORS 2

// HydraFOC motor pins
// Note: first 3 pins are pwm, second 3 are enable pins
const uint8_t focMotorPins[NUM_FOC_MOTORS][6] =
{
    {18, 46, 10, 3, 9, 11}, // Motor 0
    {21, 48, 35, 47, 45, 36} // Motor 1
};

// Driver reset and sleep pins
#define focDriverResetPin 41
#define focDriverSleepPin 42

// Current sensing pins (2 phases per motor)
const uint8_t focCurrentPins[NUM_FOC_MOTORS][2] =
{
    {17, 16}, // Motor 0
    {7, 6} // Motor 1
};

////////////////////////////////////////////////////////////////////////////
// IMU configuration
////////////////////////////////////////////////////////////////////

#define IMU_ACCEL_RANGE MPU6050_RANGE_2_G
#define IMU_GYRO_RANGE MPU6050_RANGE_250_DEG
#define IMU_FILTER_BAND MPU6050_BAND_260_HZ

#endif // HYDRA_FOC_CONFIG_H