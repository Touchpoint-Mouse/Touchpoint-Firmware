/*
  Button.h - Library for handling button events and debouncing
  Created by Carson G. Ray, August 12 2022.
*/

#ifndef Button_h
#define Button_h

#include "Arduino.h"

enum class PullMode {
	NONE = 0,
	PULLUP = 1
};

class Button {
	private:
		//Button input pin
		int pin = -1;
		
		//Debounce time
		int debounce = 3;

		//Raw button state
		bool rawstate = LOW;

		//Previous button state
		bool prevstate = LOW;

		//Debounced button state
		bool bouncestate = LOW;

		//Fallback state for debounce
		bool fallback = LOW;

		//Pullup/pulldown state
		PullMode pullMode = PullMode::NONE;

		//Current debounce start time
		int bounceStart;

		//Previous pulse start time
		int prevStart;

		//Current pulse start time
		int pulseStart;

		void updatePulse();

	public:
		Button();
		Button(int _debounce);
		Button(int _debounce, PullMode _pullMode);

		void attach(int _pin);
		void update();

		bool state();
		bool state(bool bounce);

		bool change();
		bool change(bool bounce);

		bool changeTo(bool target);
		bool changeTo(bool target, bool bounce);

		int pulseTime();
		int pulseTime(bool bounce);

		int pulse();
		int pulse(bool target);
};

#endif
