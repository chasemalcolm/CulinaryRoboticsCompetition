#include <Arduino.h>
#include "motor_control.h"
#include "pid.h"
#include "encodedVerticalSlides.h"
#include "encoder.h"
#include "configServer.h"

#define VERTICAL_HOME_POWER 0.675
#define VERTICAL_HOME_MILLIS 1000

#define VERTICAL_ENCODER_PIN_A 36
#define VERTICAL_ENCODER_PIN_B 35

#define HOMING_SWITCH_PIN 13

#define TOGGLE_DISTANCE 15 
double DOWN_POWER = -0.625;
double UP_POWER = 0.725;

bool vertical_sliding_to_set_position;

enum VerticalState {
    HOMING,
    ACTIVE_POSITION_UP,
    ACTIVE_POSITION_DOWN,
    MOVING_DOWN,
    MOVING_UP,
    IDLE
};

void updateVerticalEncoder();
Encoder verticalEncoder(VERTICAL_ENCODER_PIN_A, VERTICAL_ENCODER_PIN_B, updateVerticalEncoder);
void IRAM_ATTR updateVerticalEncoder() {
  verticalEncoder.update();
}

PIDGains upInitialGains = {0.7, 0.05, 0.004, 0.65};
double VERTICAL_MAX_JERK_UP = 0.08;
double upIntegralCap = 0.7;
PIDGains downInitialGains = {0.5, 0.03, 0.005, 0.65};
double VERTICAL_MAX_JERK_DOWN = 10000;
double downIntegralCap = 0.6;

PIDController upPID(upInitialGains, upIntegralCap, VERTICAL_MAX_JERK_UP, true, 1);
PIDController downPID(downInitialGains, downIntegralCap, VERTICAL_MAX_JERK_DOWN, true, 1);

enum VerticalState VerticalState = IDLE;

double verticalTargetPosition;
bool goingUp;

void homeVerticalSlides() {
    ySlide(- VERTICAL_HOME_POWER); // negative to move toward home switch'
    VerticalState = HOMING;
    vertical_sliding_to_set_position = false;
}

void initVerticalSlides() {
    vertical_sliding_to_set_position = false;
    verticalEncoder.begin();
    pinMode(HOMING_SWITCH_PIN, INPUT_PULLUP);
    homeVerticalSlides();
    addTweakableVariable("Up Overall", [](double newVal){upPID.changeGain(newVal);}, upInitialGains.gain);
    addTweakableVariable("Up Proportional", [](double newVal){upPID.changePorportionalGain(newVal);}, upInitialGains.porportionalGain);
    addTweakableVariable("Up Integral", [](double newVal){upPID.changeIntegralGain(newVal);}, upInitialGains.integralGain);
    addTweakableVariable("Up Derivative", [](double newVal){upPID.changeDerivativeGain(newVal);}, upInitialGains.derivativeGain);
    addTweakableVariable("Up Max Jerk", [](double newVal){upPID.maxJerk = newVal;}, VERTICAL_MAX_JERK_UP);
    addTweakableVariable("Up Power", [](double newVal){UP_POWER = newVal;}, UP_POWER);
    addTweakableVariable("Up Integral Cap", [](double newVal){upPID.integralCap = newVal;}, upIntegralCap);

    addTweakableVariable("Down Overall", [](double newVal){downPID.changeGain(newVal);}, downInitialGains.gain);
    addTweakableVariable("Down Proportional", [](double newVal){downPID.changePorportionalGain(newVal);}, downInitialGains.porportionalGain);
    addTweakableVariable("Down Integral", [](double newVal){downPID.changeIntegralGain(newVal);}, downInitialGains.integralGain);
    addTweakableVariable("Down Derivative", [](double newVal){downPID.changeDerivativeGain(newVal);}, downInitialGains.derivativeGain);
    addTweakableVariable("Down Max Jerk", [](double newVal){downPID.maxJerk = newVal;}, VERTICAL_MAX_JERK_DOWN);
    addTweakableVariable("Down Power", [](double newVal){DOWN_POWER = newVal;}, DOWN_POWER);
    addTweakableVariable("Down Integral Cap", [](double newVal){downPID.integralCap = newVal;}, downIntegralCap);
}

void activePositionVerticalSlides(double targetPosition_) {
    if (VerticalState == HOMING) {
        return; // don't try this until it finishes homing
    }

    verticalTargetPosition = targetPosition_;
    goingUp = verticalTargetPosition > verticalEncoder.position;
    int error = verticalTargetPosition - verticalEncoder.position;

    if (goingUp && abs(error) > TOGGLE_DISTANCE) {
        VerticalState = MOVING_UP;
    } else if (!goingUp && abs(error) > TOGGLE_DISTANCE) {
        VerticalState = MOVING_DOWN;
    } else if (goingUp) {
        VerticalState = ACTIVE_POSITION_UP;
    } else {
        VerticalState = ACTIVE_POSITION_DOWN;
    }

    upPID.reset();
    downPID.reset();
}

bool vertical_slide_to_integral_cut() {
    if (VerticalState == ACTIVE_POSITION_UP && upPID.integralDeleted) {
        return true;
    } else if (VerticalState == ACTIVE_POSITION_DOWN && downPID.integralDeleted) {
        return true;
    }

    return false;
}

void tickVerticalSlides() {
    switch (VerticalState) {
        double error;
        case HOMING:
            if (digitalRead(HOMING_SWITCH_PIN) == LOW) {
                ySlide(0); // relax the drawer slides
                VerticalState = IDLE;
                verticalEncoder.position = 0;
                activePositionVerticalSlides(90);
            }

            break;

        case MOVING_UP:
            ySlide(UP_POWER);
            error = verticalTargetPosition - verticalEncoder.position; 

            if (abs(error) <= TOGGLE_DISTANCE) {
                VerticalState = ACTIVE_POSITION_UP;
            }

            break;

        case MOVING_DOWN:
            ySlide(DOWN_POWER);
            error = verticalTargetPosition - verticalEncoder.position; 

            if (abs(error) <= TOGGLE_DISTANCE) {
                VerticalState = ACTIVE_POSITION_DOWN;
            }

            break;

        case ACTIVE_POSITION_UP:
            error = verticalTargetPosition - verticalEncoder.position; 
            ySlide(upPID.tick(error));
            break;

        case ACTIVE_POSITION_DOWN:
            error = verticalTargetPosition - verticalEncoder.position; 
            ySlide(downPID.tick(error));
            break;

        default:
            ySlide(0);
            break;
    }
}