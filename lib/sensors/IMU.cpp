/**
 * IMU.h - Library for interfacing with the BNO08x IMU sensor.
 */

#include "IMU.h"

IMU::IMU(uint8_t rstPin) : bno08x(rstPin) {}

bool IMU::begin(SPIClass* spi, uint8_t csPin, uint8_t intPin) {
  	if (!bno08x.begin_SPI(csPin, intPin, spi)) {
		return false;
 	}

	lastRotation = Matrix2f::Identity();
	initialOrient = Matrix2f::Identity();
	relativeRotation = Matrix2f::Identity();

  	return setReports();
}

void IMU::setZAxisOrientation(ZAxisOrientation orientation) {
	zAxisOrientation = orientation;
}

void IMU::resetReferenceFrame() {
	initialOrient = lastRotation;
	relativeRotation = initialOrient.transpose() * lastRotation;
}

bool IMU::setReports() {
  return bno08x.enableReport(SH2_ROTATION_VECTOR);
}

bool IMU::getRotationVector(Quaternionf& rotationVector) {
	if (bno08x.getSensorEvent(&sensorValue)) {
		rotationVector.w() = sensorValue.un.gameRotationVector.real;
		rotationVector.x() = sensorValue.un.gameRotationVector.i;
		rotationVector.y() = sensorValue.un.gameRotationVector.j;
		rotationVector.z() = sensorValue.un.gameRotationVector.k;
		return true;
	}
	return false;
}

bool IMU::getRelativeRotationMatrix(Matrix2f& relativeRotationMatrix, Quaternionf* rotationVector) {
	Quaternionf sampledRotation;
	if (!getRotationVector(sampledRotation)) {
		return false;
	}

	quaternionToZRotMatrix(sampledRotation, lastRotation);
	relativeRotation = initialOrient.transpose() * lastRotation;
	relativeRotationMatrix = relativeRotation;

	if (rotationVector != nullptr) {
		*rotationVector = sampledRotation;
	}

	return true;
}

void IMU::quaternionToZRotMatrix(const Quaternionf& quat, Matrix2f& rotMatrix) const {
	const float qw = quat.w();
	const float qz = quat.z();
	const float yawCos = (qw * qw) - (qz * qz);
	const float yawSign = (zAxisOrientation == ZAxisOrientation::Up) ? 1.0f : -1.0f;
	const float yawSin = yawSign * 2.0f * qw * qz;

	rotMatrix(0, 0) = yawCos;
	rotMatrix(0, 1) = -yawSin;
	rotMatrix(1, 0) = yawSin;
	rotMatrix(1, 1) = yawCos;
}