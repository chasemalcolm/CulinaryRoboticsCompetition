#pragma once
#include "Encoder.h"
#include "pid.h"
#include "tape_following.h"

    void initEncodedDrivetrain();

    // call this at a consistent interval for predictable PID behaviour.
    void tickDrivetrain();

    // Uses PID to avoid one motor spinning faster than the other
    // power in in range -1 - 1 but use something smaller in magnitude than 1 for best results.
    void driveStraight(double power);

    // Drive a set number of encoder ticks from the current position while maintaining current heading.
    // power is an absolute value and is automatically negated if driving backwards.
    void driveStraightToPosition(int relativePosition, double power);

    // Maintains a set position relative to the current position, specified in encoder ticks.
    // Pass a zero to get active braking, on the spot.
    void activePosition(int relativePosiitonLeft, int relativePositionRight);

    // Halts any active positioning, leaving the motors "dead"
    void relaxDrivetrain();

    // 143 position is 90 degrees
    // power is an absolute value and is automatically negated if driving backwards.
    // swing turn by halting the right wheel
    void swingTurnRight(int relativePosition, double power);

    // swing turn by halting the left wheel
    // power is an absolute value and is automatically negated if driving backwards.
    void swingTurnLeft(int relativePosition, double power);


    void turnLeftOnSpot(int relativePosition, double power);

    void turnRightOnSpot(int relativePosition, double power);

    bool drive_for_set_distance(int relative_position);

    bool drive_for_set_distance(int relative_position, int toggleDistance);

    bool drive_to_integral_cut();
