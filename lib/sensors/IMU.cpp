/**
 * IMU.h - Library for interfacing with the BNO08x IMU sensor.
 */

#include "IMU.h"

IMU::IMU(uint8_t rstPin) : bno08x(rstPin) {}

bool IMU::begin(SPIClass* spi, uint8_t csPin, uint8_t intPin) {
  	if (!bno08x.begin_SPI(csPin, intPin, spi)) {
		return false;
 	}

  	return setReports();
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