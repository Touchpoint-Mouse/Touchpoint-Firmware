/*
  RotEncoder.cpp - Library for handling rotary encoder inputs and step tracking
  Created by Carson G. Ray, August 12 2022.
  Released into the public domain.
*/

#include "Arduino.h"
#include "RotEncoder.h"

RotEncoder::RotEncoder() : click(Button()), delay(Button()) {
}

RotEncoder::RotEncoder(EncoderResolution _resolution) : click(Button()), delay(Button()), resolution(_resolution) {
}

void RotEncoder::attach(int _click, int _delay) {
	click.attach(_click);
	delay.attach(_delay);
	quadInitialized = false;
	quadAccum = 0;
	lastQuarterDir = 0;
}

void RotEncoder::update() {
	//Updates rotary encoder state

	//Updates button states
	click.update();
	delay.update();

	was_moved = false;
	boundaryEvent = 0;

	// Decode using raw pin states to avoid debounce-induced phase lag at high speed.
	const uint8_t currQuadState = (uint8_t(click.state(false)) << 1) | uint8_t(delay.state(false));

	if (!quadInitialized) {
		prevQuadState = currQuadState;
		quadInitialized = true;
		return;
	}

	static const int8_t kQuadDelta[16] = {
		0, -1,  1,  0,
		1,  0,  0, -1,
		-1, 0,  0,  1,
		0,  1, -1,  0
	};

	const uint8_t oldQuadState = prevQuadState;
	const uint8_t transition = (oldQuadState << 2) | currQuadState;
	prevQuadState = currQuadState;

	int8_t deltaQuarter = kQuadDelta[transition];
	if (deltaQuarter == 0) {
		// If both bits changed, we likely skipped one intermediate state.
		// Reconstruct a 2-quarter-step move using last known direction.
		const uint8_t changedBits = oldQuadState ^ currQuadState;
		if (changedBits == 0x03 && lastQuarterDir != 0) {
			deltaQuarter = static_cast<int8_t>(2 * lastQuarterDir);
		}
	}
	if (deltaQuarter == 0) {
		return;
	}

	if (directionSetting == Direction::CCW) {
		deltaQuarter = -deltaQuarter;
	}

	lastQuarterDir = (deltaQuarter > 0) ? 1 : -1;

	quadAccum += deltaQuarter;

	const int8_t threshold = (resolution == EncoderResolution::DOUBLE) ? 2 : 4;
	int8_t deltaStep = 0;
	if (quadAccum >= threshold) {
		deltaStep = 1;
		quadAccum -= threshold;
	} else if (quadAccum <= -threshold) {
		deltaStep = -1;
		quadAccum += threshold;
	}

	if (deltaStep == 0) {
		return;
	}

	rot_dir = (deltaStep > 0);
	const int32_t proposedNetSteps = net_steps + deltaStep;

	if (boundsEnabled && (proposedNetSteps < lowerBound || proposedNetSteps > upperBound)) {
		boundaryEvent = (deltaStep > 0) ? 1 : -1;
		quadAccum = 0;
		return;
	}

	was_moved = true;
	tot_steps++;
	net_steps = proposedNetSteps;
}

bool RotEncoder::dir() {
	//Gets direction
	return rot_dir;
}

int32_t RotEncoder::totalSteps() {
	//Gets total steps
	return tot_steps;
}

int32_t RotEncoder::netSteps() {
	//Gets net clockwise steps
	return net_steps;
}

void RotEncoder::setDirection(Direction direction) {
	directionSetting = direction;
}

void RotEncoder::setBounds(int32_t lower, int32_t upper) {
	if (lower > upper) {
		const int32_t temp = lower;
		lower = upper;
		upper = temp;
	}

	lowerBound = lower;
	upperBound = upper;
	boundsEnabled = true;

	if (net_steps < lowerBound) {
		net_steps = lowerBound;
		boundaryEvent = -1;
	} else if (net_steps > upperBound) {
		net_steps = upperBound;
		boundaryEvent = 1;
	}
}

void RotEncoder::clearBounds() {
	boundsEnabled = false;
}

void RotEncoder::reset() {
	//Resets step counts
	tot_steps = 0;
	net_steps = 0;
	boundaryEvent = 0;
	quadAccum = 0;
	quadInitialized = false;
	lastQuarterDir = 0;
}

//Events

int8_t RotEncoder::change() {
	if (!was_moved) {
		return 0;
	}
	return rot_dir ? 1 : -1;
}

int8_t RotEncoder::boundaryHitEvent() {
	return boundaryEvent;
}