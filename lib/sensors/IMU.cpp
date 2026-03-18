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

bool IMU::getRotationVector(Vector4f& rotationVector) {
	if (bno08x.getSensorEvent(&sensorValue)) {
		rotationVector[0] = sensorValue.un.gameRotationVector.real;
		rotationVector[1] = sensorValue.un.gameRotationVector.i;
		rotationVector[2] = sensorValue.un.gameRotationVector.j;
		rotationVector[3] = sensorValue.un.gameRotationVector.k;
		return true;
	}
	return false;
}