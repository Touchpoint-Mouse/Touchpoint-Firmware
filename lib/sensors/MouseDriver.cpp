#include "MouseDriver.h"

#include <Mouse.h>

void MouseDriver::begin() {
	Mouse.begin();
}

void MouseDriver::setLifted(bool lifted) {
	lifted_ = lifted;
}

void MouseDriver::handleOpticalDelta(int16_t deltaX, int16_t deltaY, bool lifted) {
	setLifted(lifted);

	if (lifted_ || (deltaX == 0 && deltaY == 0)) {
		return;
	}

	Mouse.move(clampToHid(deltaX), clampToHid(deltaY));
}

bool MouseDriver::isLifted() const {
	return lifted_;
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
