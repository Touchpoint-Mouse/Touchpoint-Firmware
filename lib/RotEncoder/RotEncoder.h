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

        // Optional bounds on net steps.
        bool boundsEnabled = false;
        int32_t lowerBound = 0;
        int32_t upperBound = 0;

        // -1: hit lower bound, 1: hit upper bound, 0: no bound hit.
        int8_t boundaryEvent = 0;

        // Quadrature decoder state
        bool quadInitialized = false;
        uint8_t prevQuadState = 0;
        int8_t quadAccum = 0;
        int8_t lastQuarterDir = 0;

public:
    RotEncoder();
    RotEncoder(EncoderResolution _resolution);

    void attach(int _click, int _delay);
    void update();

    bool dir();

    int32_t totalSteps();

    int32_t netSteps();

    void setDirection(Direction direction);
    void setBounds(int32_t lower, int32_t upper);
    void clearBounds();

    void reset();

    int8_t change();
    int8_t boundaryHitEvent();
};

#endif