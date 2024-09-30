#include <Arduino.h>
#include "motor_control.h"
// We need to use the lower-level LEDC functions since AnalogWrite has no means to disconnect an LEDC channel, so we can't use 10 total signals, as we only have 8 channels.
// See https://github.com/espressif/arduino-esp32/blob/release/v2.x/docs/source/api/ledc.rst

#define ANALOG_WRITE_MAX 255 //These two need to match
#define ANALOG_PRECISION_BITS 8

#define PWM_FREQUENCY 1000

// Duty cycle is 0-1, I think the are numbered 0-7.
void startPWM(int pin, int channel, double duty) {
  ledcSetup(channel, PWM_FREQUENCY, ANALOG_PRECISION_BITS);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, duty * ANALOG_WRITE_MAX);
}

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
#define DRIVE_LEFT_FORWARD_PIN 15
#define DRIVE_LEFT_BACKWARD_PIN 16
#define DRIVE_RIGHT_FORWARD_PIN 17
#define DRIVE_RIGHT_BACKWARD_PIN 18
#define DRIVE_MOTOR_MAX_POWER 1.0
#define DRIVE_LEFT_LEDC_CHANNEL 5
#define DRIVE_RIGHT_LEDC_CHANNEL 1

#define BED_FORWARD_PIN 47
#define BED_BACKWARD_PIN 21
#define BED_MAX_POWER 0.6
#define BED_LEDC_CHANNEL 2

#define ARM_ROTATION_FORWARD_PIN 38
#define ARM_ROTATION_BACKWARD_PIN 37
#define ARM_ROTATION_MAX_POWER 0.6
#define ARM_ROTATION_LEDC_CHANNEL 3

#define ARM_EXTENSION_FORWARD_PIN 36
#define ARM_EXTENSION_BACKWARD_PIN 35
#define ARM_EXTENSION_MAX_POWER 0.6
#define ARM_EXTENSION_LEDC_CHANNEL 4

void steer_drivetrain(double forward, double turning) {
  run_motor(DRIVE_LEFT_FORWARD_PIN, DRIVE_LEFT_BACKWARD_PIN, forward + turning, DRIVE_MOTOR_MAX_POWER, DRIVE_LEFT_LEDC_CHANNEL);
  run_motor(DRIVE_RIGHT_FORWARD_PIN, DRIVE_RIGHT_BACKWARD_PIN, forward - turning, DRIVE_MOTOR_MAX_POWER, DRIVE_RIGHT_LEDC_CHANNEL);
}

void run_bed(double power) {
  run_motor(BED_FORWARD_PIN, BED_BACKWARD_PIN, power, BED_MAX_POWER, BED_LEDC_CHANNEL);
}

void run_arm_rotation(double power) {
  run_motor(ARM_ROTATION_FORWARD_PIN, ARM_ROTATION_BACKWARD_PIN, power, ARM_ROTATION_MAX_POWER, ARM_ROTATION_LEDC_CHANNEL);
}

void run_arm_extension(double power) {
  run_motor(ARM_EXTENSION_FORWARD_PIN, ARM_EXTENSION_BACKWARD_PIN, power, ARM_EXTENSION_MAX_POWER, ARM_EXTENSION_LEDC_CHANNEL);
}

void init_motor_control() {
  pinMode(DRIVE_LEFT_FORWARD_PIN, OUTPUT);
  pinMode(DRIVE_LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(DRIVE_RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(DRIVE_RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(BED_FORWARD_PIN, OUTPUT);
  pinMode(BED_BACKWARD_PIN, OUTPUT);
  pinMode(ARM_EXTENSION_FORWARD_PIN, OUTPUT);
  pinMode(ARM_EXTENSION_BACKWARD_PIN, OUTPUT);
  pinMode(ARM_ROTATION_FORWARD_PIN, OUTPUT);
  pinMode(ARM_ROTATION_BACKWARD_PIN, OUTPUT);
}