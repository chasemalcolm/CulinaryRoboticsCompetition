#include <Arduino.h>
#include "motor_control.h"
#include "pid.h"
#include "encodedHorizontalSlide.h"
#include "encoder.h"
#include "configServer.h"

#define HORIZONTAL_HOME_POWER 0.6
#define HORIZONTAL_HOME_MILLIS 750

#define HORIZONTAL_ENCODER_PIN_A 41
#define HORIZONTAL_ENCODER_PIN_B 42

#define HORIZONTAL_POWER 0.7
#define HORIZONTAL_TOGGLE_DISTANCE 15

bool horizontal_sliding_to_set_position;

enum HorizontalState {
    HOMING,
    ACTIVE_POSITION,
    IDLE
};

void updateHorizontalEncoder();
Encoder horizontalEncoder(HORIZONTAL_ENCODER_PIN_A, HORIZONTAL_ENCODER_PIN_B, updateHorizontalEncoder);
void IRAM_ATTR updateHorizontalEncoder() {
  horizontalEncoder.update();
}

PIDGains horizontalInitialGains = {0.6, 0.3, 1, 2};
#define HORIZONTAL_INITIAL_MAX_JERK 0.015
double horizontalIntegralCap = 0.23;

PIDController horizontalPID(horizontalInitialGains, horizontalIntegralCap, HORIZONTAL_INITIAL_MAX_JERK, true, 2);

enum HorizontalState HorizontalState = IDLE;

long horizontalHomeStartMillis;

double horizontalTargetPosition;

void homeHorizontalSlide() {
    xSlide(- HORIZONTAL_HOME_POWER); // negative to move toward home
    horizontalHomeStartMillis = millis();
    HorizontalState = HOMING;
}

void initHorizontalSlide() {
    horizontal_sliding_to_set_position = false;
    horizontalEncoder.begin();
    homeHorizontalSlide();

    addTweakableVariable("horizontal Overall", [](double newVal){horizontalPID.changeGain(newVal);}, horizontalInitialGains.gain);
    addTweakableVariable("horizontal Proportional", [](double newVal){horizontalPID.changePorportionalGain(newVal);}, horizontalInitialGains.porportionalGain);
    addTweakableVariable("horizontal Integral", [](double newVal){horizontalPID.changeIntegralGain(newVal);}, horizontalInitialGains.integralGain);
    addTweakableVariable("horizontal Derivative", [](double newVal){horizontalPID.changeDerivativeGain(newVal);}, horizontalInitialGains.derivativeGain);
    addTweakableVariable("horizontal Max Jerk", [](double newVal){horizontalPID.maxJerk = newVal;}, HORIZONTAL_INITIAL_MAX_JERK);
    addTweakableVariable("horizontal Integral cap", [](double newVal){horizontalPID.integralCap = newVal;}, horizontalIntegralCap);
}

void activePositionHorizontalSlide(double targetPosition_) {
    if (HorizontalState == HOMING) {
        return; // don't try this at home
    }
    HorizontalState = ACTIVE_POSITION;
    horizontalTargetPosition = targetPosition_;

    horizontalPID.reset();
}

bool horizontal_slide_to_integral_cut() {
    if (HorizontalState == ACTIVE_POSITION && horizontalPID.integralDeleted) {
        return true;
    }

    return false;
}

void tickHorizontalSlide() {
    switch (HorizontalState) {
        double error, correctionSignal;
        case HOMING:
            if (millis() >= horizontalHomeStartMillis + HORIZONTAL_HOME_MILLIS) {
                HorizontalState = ACTIVE_POSITION;
                horizontalEncoder.position = 0;
                activePositionHorizontalSlide(245);
            }
            break;
        case ACTIVE_POSITION:
            error = horizontalTargetPosition - horizontalEncoder.position; 
            if (error > HORIZONTAL_TOGGLE_DISTANCE) {
                correctionSignal = HORIZONTAL_POWER;
            } else if (error < -HORIZONTAL_TOGGLE_DISTANCE) {
                correctionSignal = -HORIZONTAL_POWER;
            } else {
                correctionSignal = horizontalPID.tick(error);
            }

            xSlide(correctionSignal);
            break;
        default:
            xSlide(0);
            break;
    }
}