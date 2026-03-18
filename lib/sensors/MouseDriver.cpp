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
	prevScrollSteps = scrollWheel.cw_steps();
	prevZoomSteps = zoomWheel.cw_steps();
}

MouseDriver::SensorReadings MouseDriver::getSensorReadings() const {
	return sensorReadings;
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

void MouseDriver::setOpticalRotation(OpticalRotation rotation) {
	opticalRotation = rotation;
}

void MouseDriver::setScrollClockwisePositive(bool clockwisePositive) {
	scrollClockwisePositive = clockwisePositive;
}

void MouseDriver::setZoomClockwisePositive(bool clockwisePositive) {
	zoomClockwisePositive = clockwisePositive;
}

void MouseDriver::update() {
	cycleMoveX = 0;
	cycleMoveY = 0;
	cycleWheel = 0;

	handleButtons();
	handleWheels();

	const uint32_t nowMs = millis();
	if ((nowMs - lastOpticalPollMs) >= 10) {
		lastOpticalPollMs = nowMs;
		handleOptical();
	}

	if ((nowMs - lastImuPollMs) >= 10) {
		lastImuPollMs = nowMs;
		handleImu();
	}

	if (cycleMoveX != 0 || cycleMoveY != 0 || cycleWheel != 0) {
		Mouse.move(clampToHid(cycleMoveX), clampToHid(cycleMoveY), clampToHid(cycleWheel));
	}
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

void MouseDriver::applyOpticalRotation(int16_t inX, int16_t inY, int16_t& outX, int16_t& outY) const {
	switch (opticalRotation) {
		case OpticalRotation::Deg90:
			outX = inY;
			outY = static_cast<int16_t>(-inX);
			break;
		case OpticalRotation::Deg180:
			outX = static_cast<int16_t>(-inX);
			outY = static_cast<int16_t>(-inY);
			break;
		case OpticalRotation::Deg270:
			outX = static_cast<int16_t>(-inY);
			outY = inX;
			break;
		case OpticalRotation::Deg0:
		default:
			outX = inX;
			outY = inY;
			break;
	}
}

void MouseDriver::handleButtons() {
	leftButton.update();
	rightButton.update();
	sensorReadings.leftPressed = leftButton.state() == HIGH;
	sensorReadings.rightPressed = rightButton.state() == HIGH;

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

	sensorReadings.scrollSteps = scrollSteps;
	sensorReadings.zoomSteps = zoomSteps;

	prevScrollSteps = scrollSteps;
	prevZoomSteps = zoomSteps;

	if (scrollWheel.hasMoved()) {
		int16_t scrollStep = scrollWheel.dir() ? 1 : -1;
		if (!scrollClockwisePositive) {
			scrollStep = static_cast<int16_t>(-scrollStep);
		}
		const int16_t scaledScroll = applySensitivity(scrollStep, scrollSensitivity);
		cycleWheel += scaledScroll;
	}

	if (zoomWheel.hasMoved()) {
		int16_t zoomStep = zoomWheel.dir() ? 1 : -1;
		if (!zoomClockwisePositive) {
			zoomStep = static_cast<int16_t>(-zoomStep);
		}
		const int16_t scaledZoom = applySensitivity(zoomStep, zoomSensitivity);
		cycleMoveX += scaledZoom;
	}
}

void MouseDriver::handleOptical() {
	OpticalSensor::MotionData motionData;
	if (!opticalSensor.poll(motionData)) {
		sensorReadings.opticalValid = false;
		return;
	}

	sensorReadings.optical = motionData;
	sensorReadings.opticalValid = true;

	lifted = !motionData.onSurface;
	sensorReadings.lifted = lifted;
	if (lifted || !motionData.hasMotion) {
		return;
	}

	if (motionData.deltaX == 0 && motionData.deltaY == 0) {
		return;
	}

	const int16_t scaledX = applySensitivity(motionData.deltaX, pointerSensitivity);
	const int16_t scaledY = applySensitivity(motionData.deltaY, pointerSensitivity);
	int16_t rotatedX = 0;
	int16_t rotatedY = 0;
	applyOpticalRotation(scaledX, scaledY, rotatedX, rotatedY);
	cycleMoveX += rotatedX;
	cycleMoveY += rotatedY;
}

void MouseDriver::handleImu() {
	Eigen::Vector4f rotation;
	if (imu.getRotationVector(rotation)) {
		lastRotation = rotation;
		sensorReadings.imuRotation = rotation;
		sensorReadings.imuValid = true;
	}
}
