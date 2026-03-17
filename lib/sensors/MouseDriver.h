#ifndef MOUSE_DRIVER_H
#define MOUSE_DRIVER_H

#include <Arduino.h>

class MouseDriver {
public:
	void begin();
	void setLifted(bool lifted);
	void handleOpticalDelta(int16_t deltaX, int16_t deltaY, bool lifted);
	bool isLifted() const;

private:
	bool lifted_ = true;

	int8_t clampToHid(int16_t value) const;
};

#endif // MOUSE_DRIVER_H
