#include "MouseDriver.h"

#include <Mouse.h>
#include <math.h>

MouseDriver::MouseDriver(
	OpticalSensor& opticalSensor,
	IMU& imu,
	RotEncoder& scrollWheel,
	RotEncoder& zoomWheel,
	Button& leftButton,
	Button& rightButton
) :
	opticalSensor(opticalSensor),
	imu(imu),
	scrollWheel(scrollWheel),
	zoomWheel(zoomWheel),
	leftButton(leftButton),
	rightButton(rightButton) {}

void MouseDriver::begin() {
	Mouse.begin();
}

void MouseDriver::setPointerSensitivity(float sensitivity) {
	if (sensitivity > 0.0f) {
		pointerSensitivity = sensitivity;
	}
}

void MouseDriver::setScrollSensitivity(float sensitivity) {
	if (sensitivity > 0.0f) {
		scrollSensitivity = sensitivity;
	}
}

void MouseDriver::setZoomSensitivity(float sensitivity) {
	if (sensitivity > 0.0f) {
		zoomSensitivity = sensitivity;
	}
}

void MouseDriver::update() {
	handleButtons();
	handleWheels();
	handleOptical();
	handleImu();
}

int8_t MouseDriver::clampToHid(int16_t value) const {
	if (value > 127) {
		return 127;
	}
	if (value < -127) {
		return -127;
	}
	return static_cast<int8_t>(value);
}

int16_t MouseDriver::applySensitivity(int16_t value, float sensitivity) const {
	const float scaled = static_cast<float>(value) * sensitivity;
	if (scaled > 32767.0f) {
		return 32767;
	}
	if (scaled < -32768.0f) {
		return -32768;
	}
	return static_cast<int16_t>(lroundf(scaled));
}

void MouseDriver::handleButtons() {
	leftButton.update();
	rightButton.update();

	if (leftButton.changeTo(HIGH)) {
		Mouse.press(MOUSE_LEFT);
	} else if (leftButton.changeTo(LOW)) {
		Mouse.release(MOUSE_LEFT);
	}

	if (rightButton.changeTo(HIGH)) {
		Mouse.press(MOUSE_RIGHT);
	} else if (rightButton.changeTo(LOW)) {
		Mouse.release(MOUSE_RIGHT);
	}
}

void MouseDriver::handleWheels() {
	scrollWheel.update();
	zoomWheel.update();

	const int scrollSteps = scrollWheel.cw_steps();
	const int zoomSteps = zoomWheel.cw_steps();

	const int scrollDelta = scrollSteps - prevScrollSteps;
	const int zoomDelta = zoomSteps - prevZoomSteps;

	prevScrollSteps = scrollSteps;
	prevZoomSteps = zoomSteps;

	if (scrollDelta != 0) {
		const int16_t scaledScroll = applySensitivity(static_cast<int16_t>(scrollDelta), scrollSensitivity);
		Mouse.move(0, 0, clampToHid(scaledScroll));
	}

	if (zoomDelta != 0) {
		const int16_t scaledZoom = applySensitivity(static_cast<int16_t>(zoomDelta), zoomSensitivity);
		Mouse.move(clampToHid(scaledZoom), 0, 0);
	}
}

void MouseDriver::handleOptical() {
	OpticalSensor::MotionData motionData;
	if (!opticalSensor.poll(motionData)) {
		return;
	}

	lifted = !motionData.onSurface;
	if (lifted || !motionData.hasMotion) {
		return;
	}

	if (motionData.deltaX == 0 && motionData.deltaY == 0) {
		return;
	}

	const int16_t scaledX = applySensitivity(motionData.deltaX, pointerSensitivity);
	const int16_t scaledY = applySensitivity(motionData.deltaY, pointerSensitivity);
	Mouse.move(clampToHid(scaledX), clampToHid(scaledY), 0);
}

void MouseDriver::handleImu() {
	Eigen::Vector4f rotation;
	if (imu.getRotationVector(rotation)) {
		lastRotation = rotation;
	}
}
