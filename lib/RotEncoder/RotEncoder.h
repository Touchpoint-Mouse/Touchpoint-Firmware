/*
  RotEncoder.h - Library for handling rotary encoder inputs and step tracking
  Created by Carson G. Ray, August 12 2022.
  Released into the public domain.
*/

#ifndef RotEncoder_h
#define RotEncoder_h

#include "Arduino.h"
#include <Button.h>

class RotEncoder {
    private:
        //Click button
        Button click;

        //Delay button
        Button delay;

        //Enocoder step event
        bool was_moved = false;

        //Total steps of encoder
        int tot_steps = 0;

	  	//Net clockwise steps of encoder
        int net_steps = 0;

        //Current rotational direction (cw is true)
        bool rot_dir = true;

    public:
        RotEncoder(int _click, int _delay);
        RotEncoder(int _click, int _delay, int _bounce);

        void update();

		bool dir();

		int steps();

		int cw_steps();

		void reset();

        bool hasMoved();

        bool hasMovedInDir(bool _dir);
};

#endif