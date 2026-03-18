#ifndef MOUSE_DRIVER_H
#define MOUSE_DRIVER_H

#include <Arduino.h>
#include <ArduinoEigen.h>

#include "OpticalSensor.h"
#include "IMU.h"
#include <RotEncoder.h>
#include <Button.h>

class MouseDriver {
public:
    enum class OpticalRotation {
        Deg0 = 0,
        Deg90 = 90,
        Deg180 = 180,
        Deg270 = 270
    };

    struct SensorReadings {
        OpticalSensor::MotionData optical;
        bool opticalValid = false;
        Eigen::Vector4f imuRotation = Eigen::Vector4f::Zero();
        bool imuValid = false;
        int scrollSteps = 0;
        int zoomSteps = 0;
        bool leftPressed = false;
        bool rightPressed = false;
        bool lifted = true;
    };

    MouseDriver(OpticalSensor& opticalSensor, IMU& imu, RotEncoder& scrollWheel, RotEncoder& zoomWheel, Button& leftButton, Button& rightButton);
	void begin();
    void update();
    SensorReadings getSensorReadings() const;
    void setPointerSensitivity(float sensitivity);
    void setScrollSensitivity(float sensitivity);
    void setZoomSensitivity(float sensitivity);
    void setOpticalRotation(OpticalRotation rotation);
    void setScrollClockwisePositive(bool clockwisePositive);
    void setZoomClockwisePositive(bool clockwisePositive);

private:
    OpticalSensor& opticalSensor;
    IMU& imu;
    RotEncoder& scrollWheel;
    RotEncoder& zoomWheel;
    Button& leftButton;
    Button& rightButton;
	bool lifted = true;
    int prevScrollSteps = 0;
    int prevZoomSteps = 0;
    Eigen::Vector4f lastRotation = Eigen::Vector4f::Zero();
	float pointerSensitivity = 1.0f;
	float scrollSensitivity = 1.0f;
	float zoomSensitivity = 1.0f;
    OpticalRotation opticalRotation = OpticalRotation::Deg0;
    bool scrollClockwisePositive = true;
    bool zoomClockwisePositive = true;
    SensorReadings sensorReadings;
    int16_t cycleMoveX = 0;
    int16_t cycleMoveY = 0;
    int16_t cycleWheel = 0;

	int8_t clampToHid(int16_t value) const;
    int16_t applySensitivity(int16_t value, float sensitivity) const;
    void applyOpticalRotation(int16_t inX, int16_t inY, int16_t& outX, int16_t& outY) const;
    void handleButtons();
    void handleWheels();
    void handleOptical();
    void handleImu();
};

#endif // MOUSE_DRIVER_H
