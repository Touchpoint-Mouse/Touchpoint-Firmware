/*
  RotEncoder.cpp - Library for handling rotary encoder inputs and step tracking
  Created by Carson G. Ray, August 12 2022.
  Released into the public domain.
*/

#include "Arduino.h"
#include "RotEncoder.h"

RotEncoder::RotEncoder(int _click, int _delay) : click (Button(_click)), delay (Button(_delay)) {
}
RotEncoder::RotEncoder(int _click, int _delay, int _bounce) : click (Button(_click, _bounce)), delay (Button(_delay, _bounce)) {
}

void RotEncoder::update() {
	//Updates rotary encoder state

	//Updates button states
	click.update();
	delay.update();

	//If click has changed (rising or falling edge)
	if (click.change()) {
		//Sets click event to true
		was_moved = true;

		//Increments total steps
		tot_steps++;

		//Direction is clockwise (true) if click and delay are not equal
		rot_dir = (click.state() == delay.state());

		//Increments or decrements net steps (cw (true) is positive)
		net_steps += int(rot_dir)*2 - 1;
	} else {
		was_moved = false;
	}
}

bool RotEncoder::dir() {
	//Gets direction
	return rot_dir;
}

int RotEncoder::steps() {
	//Gets total steps
	return tot_steps;
}

int RotEncoder::cw_steps() {
	//Gets net clockwise steps
	return net_steps;
}

void RotEncoder::reset() {
	//Resets step counts
	tot_steps = 0;
	net_steps = 0;
}

//Events

bool RotEncoder::hasMoved() {
	return was_moved;
}

bool RotEncoder::hasMovedInDir(bool _dir) {
	//If click was in certain direction
	return was_moved && (dir() == _dir);
}