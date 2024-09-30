#pragma once

// call in setup()
void init_tape_following(int sensor_threshold);

// these 3 should be called very frequently if they're being used
bool left_detected();

bool right_detected();

void follow_main_line(double forward_power, double turning_power);

// FRONT_LEFT_SENSOR 6
// FRONT_RIGHT_SENSOR 8
// LEFT_SENSOR 4
// RIGHT_SENSOR 7
// CENTER_SENSOR 5
bool onTape(int PIN);

