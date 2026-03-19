/**
 * IMU.h - Library for interfacing with the BNO08x IMU sensor.
 */

#include "IMU.h"

IMU::IMU(uint8_t rstPin) : bno08x(rstPin) {}

bool IMU::begin(SPIClass* spi, uint8_t csPin, uint8_t intPin) {
	if (!bno08x.begin_SPI(csPin, intPin, spi)) {
		return false;
	}

	prevOrient = Quaternionf::Identity();
	currOrient = Quaternionf::Identity();
	hasOrientSample = false;

	prevRotation = Matrix2f::Identity();
	currRotation = Matrix2f::Identity();
	initialOrient = Matrix2f::Identity();
	hasRotationSample = false;
	referenceResetPending = true;

	return setReports();
}

bool IMU::update() {
	if (!bno08x.getSensorEvent(&sensorValue)) {
		return false;
	}

	if (sensorValue.sensorId != SH2_ROTATION_VECTOR && sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
		return false;
	}

	Quaternionf sampledOrient = Quaternionf::Identity();
	if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
		sampledOrient.w() = sensorValue.un.rotationVector.real;
		sampledOrient.x() = sensorValue.un.rotationVector.i;
		sampledOrient.y() = sensorValue.un.rotationVector.j;
		sampledOrient.z() = sensorValue.un.rotationVector.k;
	} else {
		sampledOrient.w() = sensorValue.un.gameRotationVector.real;
		sampledOrient.x() = sensorValue.un.gameRotationVector.i;
		sampledOrient.y() = sensorValue.un.gameRotationVector.j;
		sampledOrient.z() = sensorValue.un.gameRotationVector.k;
	}

	prevOrient = currOrient;
	currOrient = sampledOrient;
	hasOrientSample = true;

	prevRotation = currRotation;

	Matrix2f sampledAbsoluteRotation = Matrix2f::Identity();
	quaternionToZRotMatrix(sampledOrient, sampledAbsoluteRotation);
	if (referenceResetPending) {
		initialOrient = sampledAbsoluteRotation;
		prevRotation = Matrix2f::Identity();
		currRotation = Matrix2f::Identity();
		referenceResetPending = false;
	} else {
		currRotation = initialOrient.transpose() * sampledAbsoluteRotation;
	}
	hasRotationSample = true;

	return true;
}

void IMU::setZAxisOrientation(ZAxisOrientation orientation) {
	zAxisOrientation = orientation;
}

void IMU::resetReferenceFrame() {
	if (!hasRotationSample) {
		referenceResetPending = true;
		return;
	}

	initialOrient = initialOrient * currRotation;
	prevRotation = Matrix2f::Identity();
	currRotation = Matrix2f::Identity();
	referenceResetPending = false;
}

bool IMU::setReports() {
  return bno08x.enableReport(SH2_ROTATION_VECTOR);
}

bool IMU::getOrient(Quaternionf& rotationVector) {
	if (!hasOrientSample) {
		return false;
	}

	rotationVector = currOrient;
	return true;
}

bool IMU::getPrevOrient(Quaternionf& rotationVector) {
	if (!hasOrientSample) {
		return false;
	}

	rotationVector = prevOrient;
	return true;
}

bool IMU::getZRotation(Matrix2f& rotationMatrix) {
	if (!hasRotationSample) {
		return false;
	}

	rotationMatrix = currRotation;
	return true;
}


bool IMU::getPrevZRotation(Matrix2f& rotationMatrix) {
	if (!hasRotationSample) {
		return false;
	}

	rotationMatrix = prevRotation;
	return true;
}

void IMU::quaternionToZRotMatrix(const Quaternionf& quat, Matrix2f& rotMatrix) const {
	float qw = quat.w();
	float qx = quat.x();
	float qy = quat.y();
	float qz = quat.z();

	const float norm = sqrtf((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz));
	if (norm <= 0.0f) {
		rotMatrix = Matrix2f::Identity();
		return;
	}

	qw /= norm;
	qx /= norm;
	qy /= norm;
	qz /= norm;

	float yaw = atan2f(2.0f * ((qw * qz) + (qx * qy)), 1.0f - 2.0f * ((qy * qy) + (qz * qz)));
	if (zAxisOrientation == ZAxisOrientation::Down) {
		yaw = -yaw;
	}

	const float yawCos = cosf(yaw);
	const float yawSin = sinf(yaw);

	rotMatrix(0, 0) = yawCos;
	rotMatrix(0, 1) = -yawSin;
	rotMatrix(1, 0) = yawSin;
	rotMatrix(1, 1) = yawCos;
}