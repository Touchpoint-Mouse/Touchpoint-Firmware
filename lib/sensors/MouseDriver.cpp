#include "MouseDriver.h"
#include <Adafruit_TinyUSB.h>
#include <math.h>

namespace {
	constexpr uint8_t kMouseButtonLeft = 0x01;
	constexpr uint8_t kMouseButtonRight = 0x02;

	uint8_t gMouseButtons = 0;
	bool gMouseButtonsDirty = false;
	uint32_t gReportsAttempted = 0;
	uint32_t gReportsSent = 0;
	uint32_t gReportsDroppedNotReady = 0;

	uint8_t const kHidReportDescriptor[] = {
		TUD_HID_REPORT_DESC_MOUSE()
	};

	Adafruit_USBD_HID gUsbHid;

	void sendMouseReport(int8_t x, int8_t y, int8_t wheel) {
		if (x == 0 && y == 0 && wheel == 0) {
			// Don't send reports with no movement unless buttons changed.
			return;
		}

		// Remote wakeup
		if (TinyUSBDevice.suspended()) {
			// Wake up host if we are in suspend mode
			// and REMOTE_WAKEUP feature is enabled by host
			TinyUSBDevice.remoteWakeup();
		}

		++gReportsAttempted;
		if (!TinyUSBDevice.mounted() || !gUsbHid.ready()) {
			++gReportsDroppedNotReady;
			return;
		}

		hid_mouse_report_t report = {
			.buttons = gMouseButtons,
			.x = x,
			.y = y,
			.wheel = wheel,
			.pan = 0
		};
		gUsbHid.sendReport(0, &report, sizeof(report));
		++gReportsSent;
	}

	void beginMouseTransport() {
		// Manual begin() is required on core without built-in support e.g. mbed rp2040
		if (!TinyUSBDevice.isInitialized()) {
			TinyUSBDevice.begin(0);
		}
		
		gUsbHid.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
		gUsbHid.setPollInterval(2);
		gUsbHid.setReportDescriptor(kHidReportDescriptor, sizeof(kHidReportDescriptor));
		gUsbHid.begin();

		// If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
		if (TinyUSBDevice.mounted()) {
			TinyUSBDevice.detach();
			delay(10);
			TinyUSBDevice.attach();
		}
	}

	void setButtonState(uint8_t buttonMask, bool pressed) {
		const uint8_t previous = gMouseButtons;
		if (pressed) {
			gMouseButtons |= buttonMask;
		} else {
			gMouseButtons &= static_cast<uint8_t>(~buttonMask);
		}

		if (gMouseButtons != previous) {
			gMouseButtonsDirty = true;
		}
	}

	bool consumeButtonsDirty() {
		const bool dirty = gMouseButtonsDirty;
		gMouseButtonsDirty = false;
		return dirty;
	}
}

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
	beginMouseTransport();
	gMouseButtons = 0;
	gMouseButtonsDirty = false;
	gReportsAttempted = 0;
	gReportsSent = 0;
	gReportsDroppedNotReady = 0;

	// Reset wheel counters so real wheel state starts at zero at begin().
	scrollWheel.reset();
	zoomWheel.reset();

	// Proxy wheel state is accumulated scaled HID steps sent to host.
	proxyScrollSent = 0;
	proxyZoomSent = 0;

	// Pointer state starts at pointer offset
	// start with zero running reconciliation error (real - proxy) in mm.
	pointerErrorMm = Vector2f::Zero();
	pointerOffsetRealMm = Vector2f::Zero();
	pointerOffsetProxyMm = Vector2f::Zero();
	hasPointerOffsetProxy = false;

	// Reset IMU reference frame state at startup.
	sensorReadings.lifted = true;
	sensorReadings.imuValid = false;
	prevRelativeImuRotation = Matrix2f::Identity();
	relativeImuRotation = Matrix2f::Identity();
	imu.resetReferenceFrame();

	opticalSensor.setCpi(cpi);
}

MouseDriver::SensorReadings MouseDriver::getSensorReadings() const {
	return sensorReadings;
}

void MouseDriver::printTransportDebug(Stream& out) const {
	out.print("USBDBG mounted=");
	out.print(TinyUSBDevice.mounted() ? 1 : 0);
	out.print(" hidReady=");
	out.print(gUsbHid.ready() ? 1 : 0);
	out.print(" attempted=");
	out.print(gReportsAttempted);
	out.print(" sent=");
	out.print(gReportsSent);
	out.print(" dropped=");
	out.println(gReportsDroppedNotReady);
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

void MouseDriver::setPointerOffset(Vector2f offset) {
	pointerOffset = offset;
	pointerOffsetRealMm = relativeImuRotation * pointerOffset;
	pointerOffsetProxyMm = pointerOffsetRealMm;
	hasPointerOffsetProxy = true;
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
	#ifdef TINYUSB_NEED_POLLING_TASK
	// Manual call tud_task since it isn't called by Core's background
	TinyUSBDevice.task();
	#endif

	// not enumerated()/mounted() yet: nothing to do
	if (!TinyUSBDevice.mounted()) {
		return;
	}

	// One combined HID report per cycle.
	cycleMoveX = 0;
	cycleMoveY = 0;
	cycleWheel = 0;

	handleButtons();
	handleWheels();
	handleImu();
	handleOptical();
	updatePointerState();

	const int8_t moveX = clampToHid(cycleMoveX);
	const int8_t moveY = clampToHid(cycleMoveY);
	const int8_t wheel = clampToHid(cycleWheel);
	if (moveX != 0 || moveY != 0 || wheel != 0 || consumeButtonsDirty()) {
		sendMouseReport(moveX, moveY, wheel);
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
		setButtonState(kMouseButtonLeft, true);
	} else if (leftButton.changeTo(LOW)) {
		setButtonState(kMouseButtonLeft, false);
	}

	if (rightButton.changeTo(HIGH)) {
		setButtonState(kMouseButtonRight, true);
	} else if (rightButton.changeTo(LOW)) {
		setButtonState(kMouseButtonRight, false);
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

void MouseDriver::handleImu() {
	if (!imu.update()) {
		sensorReadings.imuValid = false;
		return;
	}

	Quaternionf rotationVector;
	if (imu.getPrevZRotation(prevRelativeImuRotation) && imu.getZRotation(relativeImuRotation) && imu.getOrient(rotationVector)) {
		sensorReadings.imuRotation[0] = rotationVector.w();
		sensorReadings.imuRotation[1] = rotationVector.x();
		sensorReadings.imuRotation[2] = rotationVector.y();
		sensorReadings.imuRotation[3] = rotationVector.z();
		sensorReadings.imuValid = true;
	} else {
		sensorReadings.imuValid = false;
	}
}

void MouseDriver::handleOptical() {
	OpticalSensor::MotionData motionData;
	if (!opticalSensor.poll(motionData)) {
		sensorReadings.opticalValid = false;
		sensorReadings.hasMotion = false;
		return;
	}

	sensorReadings.optical = motionData;
	sensorReadings.opticalValid = true;
	sensorReadings.hasMotion = motionData.hasMotion;

	sensorReadings.wasLifted = sensorReadings.lifted;
	sensorReadings.lifted = !motionData.onSurface;
}

void MouseDriver::updatePointerState() {
	bool resetReferenceThisFrame = false;
	if (sensorReadings.wasLifted && !sensorReadings.lifted) {
		// Re-zero IMU-relative frame when touching down again.
		imu.resetReferenceFrame();
		prevRelativeImuRotation = Matrix2f::Identity();
		relativeImuRotation = Matrix2f::Identity();
		hasPointerOffsetProxy = false;
		resetReferenceThisFrame = true;
	}

    // Nothing to do if lifted
	if (sensorReadings.lifted) {
		return;
	}

    const float countsPerMm = cpi / 25.4f;

    // Add optical pointer delta if valid
    if (sensorReadings.opticalValid && sensorReadings.optical.hasMotion) {
        // Keep optical data in configured sensor frame first.
        const Vector2f sensorDelta(sensorReadings.optical.deltaX, sensorReadings.optical.deltaY);

        // Apply optical rotation to raw sensor delta to get configured sensor frame delta.
        Vector2f configuredSensorDelta;
        applyOpticalRotation(sensorDelta, configuredSensorDelta);
        // Apply pointer offset in IMU reference frame in headless mode
        const Vector2f pointerCountsDelta = headlessModeEnabled
            ? (relativeImuRotation * configuredSensorDelta)
            : configuredSensorDelta;

        // Accumulate real movement into running error (real - proxy) in mm.
        pointerErrorMm += pointerCountsDelta / countsPerMm;
    }

	// Offset channel: absolute target tracking using real/proxy vectors.
	if (sensorReadings.imuValid && headlessModeEnabled) {
		pointerOffsetRealMm = relativeImuRotation * pointerOffset;

		if (resetReferenceThisFrame || !hasPointerOffsetProxy) {
			pointerOffsetProxyMm = pointerOffsetRealMm;
			hasPointerOffsetProxy = true;
		} else {
			const Vector2f offsetErrorMm = pointerOffsetRealMm - pointerOffsetProxyMm;
			const int32_t offsetDeltaXHid = static_cast<int32_t>(lroundf(offsetErrorMm.x() * countsPerMm));
			const int32_t offsetDeltaYHid = static_cast<int32_t>(lroundf(offsetErrorMm.y() * countsPerMm));
			const int8_t offsetStepX = clampToHid(offsetDeltaXHid);
			const int8_t offsetStepY = clampToHid(offsetDeltaYHid);

			cycleMoveX += offsetStepX;
			cycleMoveY += offsetStepY;

			const float offsetStepToMm = 1.0f / countsPerMm;
			pointerOffsetProxyMm.x() += offsetStepX * offsetStepToMm;
			pointerOffsetProxyMm.y() += offsetStepY * offsetStepToMm;
		}
	}

	// Convert error to HID deltas, clamp, then remove sent motion from error.
	const int32_t deltaXHid = static_cast<int32_t>(lroundf(pointerErrorMm.x() * pointerSensitivity));
	const int32_t deltaYHid = static_cast<int32_t>(lroundf(pointerErrorMm.y() * pointerSensitivity));
	const int8_t stepX = clampToHid(deltaXHid);
	const int8_t stepY = clampToHid(deltaYHid);

	cycleMoveX += stepX;
	cycleMoveY += stepY;

	const float stepToMm = 1.0f / pointerSensitivity;
	pointerErrorMm.x() -= stepX * stepToMm;
	pointerErrorMm.y() -= stepY * stepToMm;
}
