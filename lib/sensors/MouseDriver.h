#ifndef MOUSE_DRIVER_H
#define MOUSE_DRIVER_H

#include <Arduino.h>
#include <ArduinoEigen.h>

#include "OpticalSensor.h"
#include "IMU.h"
#include <RotEncoder.h>
#include <Button.h>

using namespace Eigen;

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
        Vector4f imuRotation = Vector4f::Zero();
        bool imuValid = false;
        int32_t scrollSteps = 0;
        int32_t zoomSteps = 0;
        bool leftPressed = false;
        bool rightPressed = false;
        bool lifted = true;
    };

    MouseDriver(OpticalSensor& opticalSensor, IMU& imu, RotEncoder& scrollWheel, RotEncoder& zoomWheel, Button& leftButton, Button& rightButton);
	void begin();
    void update();
    SensorReadings getSensorReadings() const;
    bool setCPI(uint16_t cpi);
    void setPointerSensitivity(float sensitivity);
    void setScrollSensitivity(float sensitivity);
    void setZoomSensitivity(float sensitivity);
    void setHeadlessModeEnabled(bool enabled);
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
    Matrix2f relativeImuRotation = Matrix2f::Identity();
    uint16_t cpi = 1100;

	// Running difference between real pointer position and proxy (sent) position in mm.
    Vector2f pointerErrorMm = Vector2f::Zero();

	// Proxy states track what has been sent to the host:
	// - wheel proxy is in scaled HID steps sent
	// Pointer reconciliation keeps only running error in mm.
    int32_t proxyScrollSent = 0;
    int32_t proxyZoomSent = 0;

	float pointerSensitivity = 1.0f;
	float scrollSensitivity = 1.0f;
	float zoomSensitivity = 1.0f;
    bool headlessModeEnabled = true;
    OpticalRotation opticalRotation = OpticalRotation::Deg0;
    SensorReadings sensorReadings;
    int32_t cycleMoveX = 0;
    int32_t cycleMoveY = 0;
    int32_t cycleWheel = 0;

	int8_t clampToHid(int32_t value) const;
    void applyOpticalRotation(const Vector2f& in, Vector2f& out) const;
    void updatePointerState(const Vector2f& relativeCountsDelta);
    void handleButtons();
    void handleWheels();
    void handleOptical();
    void handleImu();
};

#endif // MOUSE_DRIVER_H
