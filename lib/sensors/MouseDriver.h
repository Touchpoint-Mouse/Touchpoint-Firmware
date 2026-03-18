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
    enum class ImuZAxisOrientation {
        Up,
        Down
    };

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
    void setImuZAxisOrientation(ImuZAxisOrientation orientation);
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
    Matrix2f lastRotation = Matrix2f::Identity();
    Matrix2f initialOrient = Matrix2f::Identity();
    Matrix2f relativeImuRotation = Matrix2f::Identity();
    uint16_t cpi = 1100;

	// Real pointer state in physical millimeters.
    Vector2d realPointerMm = Vector2d::Zero();

	// Proxy states track what has been sent to the host:
	// - wheel proxy is in scaled HID steps sent
	// - pointer proxy is in pixel deltas sent
    int32_t proxyScrollSent = 0;
    int32_t proxyZoomSent = 0;
    int32_t proxyPointerXPixels = 0;
    int32_t proxyPointerYPixels = 0;

	float pointerSensitivity = 1.0f;
	float scrollSensitivity = 1.0f;
	float zoomSensitivity = 1.0f;
    bool headlessModeEnabled = true;
    ImuZAxisOrientation imuZAxisOrientation = ImuZAxisOrientation::Up;
    OpticalRotation opticalRotation = OpticalRotation::Deg0;
    SensorReadings sensorReadings;
    int32_t cycleMoveX = 0;
    int32_t cycleMoveY = 0;
    int32_t cycleWheel = 0;

	int8_t clampToHid(int32_t value) const;
    void updateRelativeImuRotation();
    void applyOpticalRotation(const Vector2f& in, Vector2f& out) const;
    void updatePointerState(const Vector2f& relativeCountsDelta);
    void handleButtons();
    void handleWheels();
    void handleOptical();
    void handleImu();
    void quaternionToZRotMatrix(const Quaternionf& quat, Matrix2f& rotMatrix) const;
};

#endif // MOUSE_DRIVER_H
