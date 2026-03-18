/*
  RotEncoder.h - Library for handling rotary encoder inputs and step tracking
  Created by Carson G. Ray, August 12 2022.
  Released into the public domain.
*/

#ifndef RotEncoder_h
#define RotEncoder_h

#include "Arduino.h"
#include <Button.h>

enum class EncoderResolution {
	SINGLE = 0,
	DOUBLE = 1
};

class RotEncoder {
public:
    enum class Direction {
        CW = 0,
        CCW = 1
    };

private:
        //Click button
        Button click;

        //Delay button
        Button delay;

        //Whether to use double resolution (counts both rising and falling edges of click)
		EncoderResolution resolution = EncoderResolution::SINGLE;
        //Enocoder step event
        bool was_moved = false;

        //Total steps of encoder
        int32_t tot_steps = 0;

	  	//Net clockwise steps of encoder
        int32_t net_steps = 0;

        //Current rotational direction (cw is true)
        bool rot_dir = true;

        //Configured positive direction for this encoder.
        Direction directionSetting = Direction::CW;

public:
    RotEncoder();
    RotEncoder(int _debounce);
    RotEncoder(int _debounce, EncoderResolution _resolution);

    void attach(int _click, int _delay);
    void update();

    bool dir();

    int32_t steps();

    int32_t netSteps();

    void setDirection(Direction direction);

    void reset();

    bool hasMoved();

    bool hasMovedInDir(bool _dir);
};

#endif