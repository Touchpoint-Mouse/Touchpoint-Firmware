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
    enum class ZAxisOrientation {
        Up,
        Down
    };

    IMU(uint8_t rstPin);
	bool begin(SPIClass* spi, uint8_t csPin, uint8_t intPin);
    void setZAxisOrientation(ZAxisOrientation orientation);
    void resetReferenceFrame();
    bool getRelativeRotationMatrix(Matrix2f& relativeRotationMatrix, Quaternionf* rotationVector = nullptr);
private:
	Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;
    ZAxisOrientation zAxisOrientation = ZAxisOrientation::Up;
    Matrix2f lastRotation = Matrix2f::Identity();
    Matrix2f initialOrient = Matrix2f::Identity();
    Matrix2f relativeRotation = Matrix2f::Identity();

    bool getRotationVector(Quaternionf& rotationVector);
    void quaternionToZRotMatrix(const Quaternionf& quat, Matrix2f& rotMatrix) const;
    bool setReports();
};

#endif // IMU_H