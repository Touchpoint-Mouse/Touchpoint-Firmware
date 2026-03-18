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
    MouseDriver(OpticalSensor& opticalSensor, IMU& imu, RotEncoder& scrollWheel, RotEncoder& zoomWheel, Button& leftButton, Button& rightButton);
	void begin();
    void update();
    void setPointerSensitivity(float sensitivity);
    void setScrollSensitivity(float sensitivity);
    void setZoomSensitivity(float sensitivity);

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

	int8_t clampToHid(int16_t value) const;
    int16_t applySensitivity(int16_t value, float sensitivity) const;
    void handleButtons();
    void handleWheels();
    void handleOptical();
    void handleImu();
};

#endif // MOUSE_DRIVER_H
