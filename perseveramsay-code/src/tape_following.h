#pragma once

#define FRONT_LEFT_SENSOR 6
#define FRONT_RIGHT_SENSOR 7
#define LEFT_SENSOR 4
#define RIGHT_SENSOR 5
#define CENTER_SENSOR 8

enum Side {LEFT, RIGHT};

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


void stop_following_main_line();

// use in tasks 
// Stops following tape when it gets to the station
bool atStation(enum Side side, int tapes_to_count);



// in loop
void tickTapeFollowing();