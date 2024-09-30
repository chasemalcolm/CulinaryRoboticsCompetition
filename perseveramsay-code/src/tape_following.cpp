#include <Arduino.h>
#include "motor_control.h"
#include "tape_following.h"
#include "configServer.h"
#include "encodedDrivetrain.h"

double follow_power;
double turn_power;

enum States {FOLLOWING_MAIN_LINE, NOT_FOLLOWING_MAIN_LINE};
enum States current_state;

enum DrivingStates {MAIN_LINE, OFF_LEFT, OFF_RIGHT};
enum DrivingStates driving_state;
int threshold;

bool at_station_in_progress;
int at_station_count;
int at_station_sensor;
bool canCount;

void init_tape_following(int sensor_threshold) {
  threshold = sensor_threshold;
  current_state = NOT_FOLLOWING_MAIN_LINE;
  driving_state = MAIN_LINE;
  pinMode(FRONT_LEFT_SENSOR, INPUT);
  pinMode(FRONT_RIGHT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  follow_power = 0;
  turn_power = 0;
  at_station_in_progress = false;
  canCount = true;

  addValueReadout("Driving State", [](){return (double) driving_state;});
  addValueReadout("Tape Following State", [](){return (double) current_state;});
  addValueReadout("Follow Power", [](){return (double) follow_power;});
  addValueReadout("Turn Power", [](){return (double) turn_power;});
}

bool left_detected() {
  return onTape(LEFT_SENSOR);
}

bool right_detected() {
  return onTape(LEFT_SENSOR);
}

void driving(enum DrivingStates driving_state_1, double forward_power, double turning_power) {
  switch(driving_state_1) {
    case MAIN_LINE: 
      steer_drivetrain(forward_power, 0);
      break;
    case OFF_LEFT:
      steer_drivetrain(forward_power, turning_power);
      break;
    case OFF_RIGHT:
      steer_drivetrain(forward_power, -turning_power);
      break;
  }
}

bool onTape(int PIN) {
  return analogRead(PIN) > threshold;
}

void follow_main_line(double forward_power, double turning_power) {
  relaxDrivetrain();
  follow_power = forward_power;
  turn_power = turning_power;
  current_state = FOLLOWING_MAIN_LINE;
}

void stop_following_main_line() {
  current_state = NOT_FOLLOWING_MAIN_LINE;
  steer_drivetrain(0, 0);
}

bool atStation(enum Side side, int tapes_to_count) {
  if (!at_station_in_progress) {
    canCount = true;
    at_station_in_progress = true;
    at_station_count = tapes_to_count;

    switch (side) {
      case LEFT:
        at_station_sensor = LEFT_SENSOR;
        break;
      case RIGHT:
        at_station_sensor = RIGHT_SENSOR;
        break;
    }
  }

  if (onTape(at_station_sensor) && canCount) {
    at_station_count--;
    canCount = false;
  }

  if (at_station_count == 0) {
    at_station_in_progress = false;
    stop_following_main_line();
    return true;
  }

  if (!onTape(at_station_sensor) && !canCount) {
    canCount = true;
  }
  
  return false;
}

void tickTapeFollowing() {

  if (!onTape(FRONT_LEFT_SENSOR) && onTape(FRONT_RIGHT_SENSOR)) {
    driving_state = OFF_LEFT;
  } 
  if (!onTape(FRONT_RIGHT_SENSOR) && onTape(FRONT_LEFT_SENSOR)) {
    driving_state = OFF_RIGHT;
  }
  if (onTape(FRONT_LEFT_SENSOR) && onTape(FRONT_RIGHT_SENSOR)) {
    driving_state = MAIN_LINE;
  }

  if (current_state == FOLLOWING_MAIN_LINE) {
    driving(driving_state, follow_power, turn_power);
  }
}