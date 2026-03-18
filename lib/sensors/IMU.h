/**
 * IMU.h - Library for interfacing with the BNO08x IMU sensor.
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <ArduinoEigen.h>

using namespace Eigen;

class IMU {
public:
    IMU(uint8_t rstPin);
	bool begin(SPIClass* spi, uint8_t csPin, uint8_t intPin);
    bool getRotationVector(Quaternionf& rotationVector);
private:
	Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;

    bool setReports();
};

#endif // IMU_H