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

	// Reset wheel counters so real wheel state starts at zero at begin().
	scrollWheel.reset();
	zoomWheel.reset();

	// Proxy wheel state is accumulated scaled HID steps sent to host.
	proxyScrollSent = 0;
	proxyZoomSent = 0;

	// Pointer states start at origin:
	// real state in mm, proxy state as sent pixel deltas.
	realPointerMm = Vector2d::Zero();
	proxyPointerXPixels = 0;
	proxyPointerYPixels = 0;

	// Reset IMU reference frame state at startup.
	lifted = true;
	sensorReadings.lifted = true;
	sensorReadings.imuValid = false;
	lastRotation = Matrix2f::Identity();
	initialOrient = Matrix2f::Identity();
	updateRelativeImuRotation();

	opticalSensor.setCpi(cpi);
}

MouseDriver::SensorReadings MouseDriver::getSensorReadings() const {
	return sensorReadings;
}

bool MouseDriver::setCPI(uint16_t cpi) {
	this->cpi = cpi;
	return opticalSensor.setCpi(cpi);
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

void MouseDriver::setHeadlessModeEnabled(bool enabled) {
	headlessModeEnabled = enabled;
}

void MouseDriver::setImuZAxisOrientation(ImuZAxisOrientation orientation) {
	imuZAxisOrientation = orientation;
	updateRelativeImuRotation();
}

void MouseDriver::setOpticalRotation(OpticalRotation rotation) {
	opticalRotation = rotation;
}

void MouseDriver::setScrollClockwisePositive(bool clockwisePositive) {
	scrollWheel.setDirection(clockwisePositive ? RotEncoder::Direction::CW : RotEncoder::Direction::CCW);
}

void MouseDriver::setZoomClockwisePositive(bool clockwisePositive) {
	zoomWheel.setDirection(clockwisePositive ? RotEncoder::Direction::CW : RotEncoder::Direction::CCW);
}

void MouseDriver::update() {
	// One combined HID report per cycle.
	cycleMoveX = 0;
	cycleMoveY = 0;
	cycleWheel = 0;

	handleButtons();
	handleWheels();
	handleImu();
	handleOptical();

	if (cycleMoveX != 0 || cycleMoveY != 0 || cycleWheel != 0) {
		Mouse.move(clampToHid(cycleMoveX), clampToHid(cycleMoveY), clampToHid(cycleWheel));
	}
}

int8_t MouseDriver::clampToHid(int32_t value) const {
	if (value > 127) {
		return 127;
	}
	if (value < -127) {
		return -127;
	}
	return static_cast<int8_t>(value);
}

void MouseDriver::updateRelativeImuRotation() {
	relativeImuRotation = initialOrient.transpose() * lastRotation;
}

void MouseDriver::applyOpticalRotation(const Vector2f& in, Vector2f& out) const {
	switch (opticalRotation) {
		case OpticalRotation::Deg90:
			out.x() = in.y();
			out.y() = -in.x();
			break;
		case OpticalRotation::Deg180:
			out.x() = -in.x();
			out.y() = -in.y();
			break;
		case OpticalRotation::Deg270:
			out.x() = -in.y();
			out.y() = in.x();
			break;
		case OpticalRotation::Deg0:
		default:
			out = in;
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

	// Absolute net encoder state is the source of truth.
	const int32_t realScrollSteps = scrollWheel.netSteps();
	const int32_t realZoomSteps = zoomWheel.netSteps();

	sensorReadings.scrollSteps = realScrollSteps;
	sensorReadings.zoomSteps = realZoomSteps;

	// Scroll reconciliation pipeline:
	// real (unscaled steps) -> compare to unscaled proxy -> rescale -> clamp -> send.
	// Proxy stores scaled HID steps sent to host.
	const float scrollProxyUnscaled = proxyScrollSent / scrollSensitivity;
	const float scrollErrorUnscaled = realScrollSteps - scrollProxyUnscaled;
	const int32_t scrollDeltaHid = static_cast<int32_t>(lroundf(scrollErrorUnscaled * scrollSensitivity));
	const int8_t scrollStepHid = clampToHid(scrollDeltaHid);
	cycleWheel += scrollStepHid;
	proxyScrollSent += scrollStepHid;

	// Zoom reconciliation follows the same model and maps to X-axis movement.
	const float zoomProxyUnscaled = proxyZoomSent / zoomSensitivity;
	const float zoomErrorUnscaled = realZoomSteps - zoomProxyUnscaled;
	const int32_t zoomDeltaHid = static_cast<int32_t>(lroundf(zoomErrorUnscaled * zoomSensitivity));
	const int8_t zoomStepHid = clampToHid(zoomDeltaHid);
	cycleMoveX += zoomStepHid;
	proxyZoomSent += zoomStepHid;
}

void MouseDriver::handleOptical() {
	OpticalSensor::MotionData motionData;
	if (!opticalSensor.poll(motionData)) {
		sensorReadings.opticalValid = false;
		return;
	}

	sensorReadings.optical = motionData;
	sensorReadings.opticalValid = true;

	const bool wasLifted = lifted;
	lifted = !motionData.onSurface;
	sensorReadings.lifted = lifted;

	if (wasLifted && !lifted) {
		// Re-zero IMU-relative frame when touching down again.
		initialOrient = lastRotation;
		updateRelativeImuRotation();
	}

	if (lifted || !motionData.hasMotion) {
		return;
	}

	if (motionData.deltaX == 0 && motionData.deltaY == 0) {
		return;
	}

	// Keep optical data in configured sensor frame first.
	const Vector2f sensorDelta(motionData.deltaX, motionData.deltaY);
    Vector2f configuredSensorDelta;
    applyOpticalRotation(sensorDelta, configuredSensorDelta);

	// Optionally transform sensor delta into IMU-relative frame before pointer adjustment.
	const Vector2f pointerCountsDelta = headlessModeEnabled
		? (relativeImuRotation * configuredSensorDelta)
		: configuredSensorDelta;
	updatePointerState(pointerCountsDelta);
}

void MouseDriver::handleImu() {
	Quaternionf rotation;
	if (imu.getRotationVector(rotation)) {
		quaternionToZRotMatrix(rotation, lastRotation);
		updateRelativeImuRotation();
		sensorReadings.imuRotation[0] = rotation.w();
		sensorReadings.imuRotation[1] = rotation.x();
		sensorReadings.imuRotation[2] = rotation.y();
		sensorReadings.imuRotation[3] = rotation.z();
		sensorReadings.imuValid = true;
	} else {
		sensorReadings.imuValid = false;
	}
}

void MouseDriver::updatePointerState(const Vector2f& relativeCountsDelta) {
	// Real pointer path in highest available precision (mm).
	const double countsPerMm = cpi / 25.4;
	realPointerMm += relativeCountsDelta.cast<double>() / countsPerMm;

	// Proxy reconciliation:
	// proxy stores sent pixels; unscale to mm, compare against real mm,
	// then rescale/clamp for this HID report.
	const float pointerSensitivityF = pointerSensitivity;
	const float countsPerMmF = countsPerMm;
	const float proxyXmm = (proxyPointerXPixels / pointerSensitivityF) / countsPerMmF;
	const float proxyYmm = (proxyPointerYPixels / pointerSensitivityF) / countsPerMmF;
	const float errorXmm = realPointerMm.x() - proxyXmm;
	const float errorYmm = realPointerMm.y() - proxyYmm;

	const float errorXCounts = errorXmm * countsPerMmF;
	const float errorYCounts = errorYmm * countsPerMmF;
	const int32_t deltaXHid = static_cast<int32_t>(lroundf(errorXCounts * pointerSensitivityF));
	const int32_t deltaYHid = static_cast<int32_t>(lroundf(errorYCounts * pointerSensitivityF));
	const int8_t stepX = clampToHid(deltaXHid);
	const int8_t stepY = clampToHid(deltaYHid);

	cycleMoveX += stepX;
	cycleMoveY += stepY;
	proxyPointerXPixels += stepX;
	proxyPointerYPixels += stepY;
}

void MouseDriver::quaternionToZRotMatrix(const Quaternionf& quat, Matrix2f& rotMatrix) const {
	// Build a 2D yaw-only rotation matrix from quaternion components.
	const float qw = quat.w();
	const float qz = quat.z();
	const float yawCos = (qw * qw) - (qz * qz);
	const float yawSign = (imuZAxisOrientation == ImuZAxisOrientation::Up) ? 1.0f : -1.0f;
	const float yawSin = yawSign * 2.0f * qw * qz;

	rotMatrix(0, 0) = yawCos;
	rotMatrix(0, 1) = -yawSin;
	rotMatrix(1, 0) = yawSin;
	rotMatrix(1, 1) = yawCos;
}
