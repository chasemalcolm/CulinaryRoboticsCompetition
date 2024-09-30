#include <Arduino.h>
#include "motor_control.h"
#include <s3servo.h>

// We need to use the lower-level LEDC functions since AnalogWrite has no means to disconnect an LEDC channel, so we can't use 10 total signals, as we only have 8 channels.
// See https://github.com/espressif/arduino-esp32/blob/release/v2.x/docs/source/api/ledc.rst

#define ANALOG_WRITE_MAX 255
#define ANALOG_PRECISION_BITS 8
#define PWM_FREQUENCY 1000

// Duty cycle is 0-1, I think the are numbered 0-7.
void startPWM(int pin, int channel, double duty) {
  ledcSetup(channel, PWM_FREQUENCY, ANALOG_PRECISION_BITS);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, duty * ANALOG_WRITE_MAX);
}

// pwm's the appripriate H-Bridge pin
// maxPower is the maximum duty cycle (0 - 1)
// power is the portion of that maximum (-1 - 1)
void run_motor(int pinForward, int pinBackward, double power, double maxPower, int LEDCChannel) {
  double outputDuty = maxPower * min(abs(power), 1.0);
  if (power > 0) {
    ledcDetachPin(pinBackward); // Usethis instead of AnalogWrite(pin, 0) to free up an LEDC channel
    digitalWrite(pinBackward, LOW);
    startPWM(pinForward, LEDCChannel, outputDuty);
  }
  else {
    ledcDetachPin(pinForward);
    digitalWrite(pinForward, LOW);
    startPWM(pinBackward, LEDCChannel, outputDuty);
  }
}

// both inputs are in range -1 - 1
// turning is the strength of a right turn
#define DRIVE_LEFT_FORWARD_PIN 16
#define DRIVE_LEFT_BACKWARD_PIN 15
#define DRIVE_LEFT_CHANNEL 0
#define DRIVE_RIGHT_FORWARD_PIN 17
#define DRIVE_RIGHT_BACKWARD_PIN 18
#define DRIVE_RIGHT_CHANNEL 1
#define DRIVE_MOTOR_MAX_POWER 1.0
#define LEFT_DRIVE_MOTOR_MAX_POWER 1.0
#define RIGHT_DRIVE_MOTOR_MAX_POWER 1.0

#define HORIZONTAL_FORWARD_PIN 39
#define HORIZONTAL_BACKWARD_PIN 40
#define HORIZONTAL_CHANNEL 2
#define HORIZONTAL_MAX_POWER 0.75

#define VERTICAL_FORWARD_PIN 37
#define VERTICAL_BACKWARD_PIN 38
#define VERTICAL_CHANNEL 3
#define VERTICAL_MAX_POWER 1

s3servo fryPryServo;
#define FRY_PRY_PIN 47
#define FRY_PRY_MAX_SPEED 3
#define FRY_PRY_CHANNEL 4
#define FRY_PRY_ZERO_POSITION 305
double currentFryPryPosition;
double desiredFryPryPosition;
double fryPryPower;

s3servo spatulaServo;
#define SPATULA_PIN 21
#define SPATULA_MAX_SPEED 3
#define SPATULA_CHANNEL 5
#define SPATULA_ZERO_POSITION 169


void run_drive_motors(double leftPower, double rightPower) {
  run_motor(DRIVE_LEFT_FORWARD_PIN, DRIVE_LEFT_BACKWARD_PIN, leftPower, DRIVE_MOTOR_MAX_POWER, DRIVE_LEFT_CHANNEL);
  run_motor(DRIVE_RIGHT_FORWARD_PIN, DRIVE_RIGHT_BACKWARD_PIN, rightPower, DRIVE_MOTOR_MAX_POWER, DRIVE_RIGHT_CHANNEL);
}

void steer_drivetrain(double forward, double turning) {
  run_motor(DRIVE_LEFT_FORWARD_PIN, DRIVE_LEFT_BACKWARD_PIN, forward + turning, LEFT_DRIVE_MOTOR_MAX_POWER, DRIVE_LEFT_CHANNEL);
  run_motor(DRIVE_RIGHT_FORWARD_PIN, DRIVE_RIGHT_BACKWARD_PIN, forward - turning, RIGHT_DRIVE_MOTOR_MAX_POWER, DRIVE_RIGHT_CHANNEL);
}

void xSlide(double power) {
  run_motor(HORIZONTAL_FORWARD_PIN, HORIZONTAL_BACKWARD_PIN, power, HORIZONTAL_MAX_POWER, HORIZONTAL_CHANNEL);
}

void ySlide(double power) {
  run_motor(VERTICAL_FORWARD_PIN, VERTICAL_BACKWARD_PIN, power, VERTICAL_MAX_POWER, VERTICAL_CHANNEL);
}

void pryFry(double position, double power) {
  desiredFryPryPosition = position;
  fryPryPower = power * FRY_PRY_MAX_SPEED;
}

void tickFryPry() {
  if (desiredFryPryPosition > currentFryPryPosition) {
    currentFryPryPosition = min(currentFryPryPosition + fryPryPower, desiredFryPryPosition);
  } else {
    currentFryPryPosition = max(currentFryPryPosition - fryPryPower, desiredFryPryPosition);
  }

  fryPryServo.write(currentFryPryPosition + FRY_PRY_ZERO_POSITION);
}

void moveSpatula(double position) {
  spatulaServo.write(position + SPATULA_ZERO_POSITION);
}

void init_servos() {
  fryPryServo.attach(FRY_PRY_PIN, FRY_PRY_CHANNEL);
  fryPryServo.write(FRY_PRY_ZERO_POSITION - 130);
  currentFryPryPosition = -130;
  spatulaServo.attach(SPATULA_PIN, SPATULA_CHANNEL);
}

void init_motor_control() {
  pinMode(DRIVE_LEFT_FORWARD_PIN, OUTPUT);
  pinMode(DRIVE_LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(DRIVE_RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(DRIVE_RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(HORIZONTAL_FORWARD_PIN, OUTPUT);
  pinMode(HORIZONTAL_BACKWARD_PIN, OUTPUT);
  pinMode(VERTICAL_FORWARD_PIN, OUTPUT);
  pinMode(VERTICAL_BACKWARD_PIN, OUTPUT);
  init_servos();
}