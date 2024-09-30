#include <Arduino.h>
#include "motor_control.h"
#include "tape_following.h"

#define FRONT_LEFT_SENSOR 6
#define FRONT_RIGHT_SENSOR 8
#define LEFT_SENSOR 7
#define RIGHT_SENSOR 4
#define CENTER_SENSOR 5

enum DrivingStates {MAIN_LINE, OFF_LEFT, OFF_RIGHT, TURN_LEFT, TURN_RIGHT, OFF_BOTH};
int threshold;

void init_tape_following(int sensor_threshold) {
  init_motor_control();
  threshold = sensor_threshold;
  pinMode(FRONT_LEFT_SENSOR, INPUT);
  pinMode(FRONT_RIGHT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

bool left_detected() {
  return onTape(LEFT_SENSOR);
}

bool right_detected() {
  return onTape(LEFT_SENSOR);
}

void driving(enum DrivingStates driving_state, double forward_power, double turning_power) {
  switch(driving_state) {
    case MAIN_LINE: 
      steer_drivetrain(forward_power, 0);
      break;
    case OFF_LEFT:
      steer_drivetrain(forward_power, turning_power);
      break;
    case OFF_RIGHT:
      steer_drivetrain(forward_power, -turning_power);
      break;
    case TURN_LEFT:
      steer_drivetrain(0, 0);
      break;
    case TURN_RIGHT:
      steer_drivetrain(0, 0);
      break;
    case OFF_BOTH: 
      steer_drivetrain(0, 0);
      break;
  }
}

bool onTape(int PIN) {
  return analogRead(PIN) > threshold;
}

void follow_main_line(double forward_power, double turning_power) {
  enum DrivingStates driving_state = OFF_BOTH;
  if (!onTape(FRONT_LEFT_SENSOR)) {
    driving_state = OFF_LEFT;
  } 
  if (!onTape(FRONT_RIGHT_SENSOR)) {
    driving_state = OFF_RIGHT;
  }
  if (onTape(FRONT_LEFT_SENSOR) && onTape(FRONT_RIGHT_SENSOR)) {
    driving_state = MAIN_LINE;
  }
  if (!onTape(FRONT_LEFT_SENSOR) && !onTape(FRONT_RIGHT_SENSOR)) {
    driving_state = OFF_BOTH;
  }
  driving(driving_state, forward_power, turning_power);
}
