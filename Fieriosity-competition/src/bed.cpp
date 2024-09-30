#include <Arduino.h>
#include "bed.h"
#include "motor_control.h"
#include "pid.h"
#include "configServer.h"
#include <I2CEncoder.h>

#define BED_HOME_MOTOR_POWER 0.2
#define BED_HOME_POWER 0.6
#define BED_ACCEPTABLE_ERROR 0.03

#define BED_HOME_MILLIS 1000
I2CEncoder bedEncoder;

enum BedState {
    HOMING,
    ACTIVE_POSITION,
    IDLE
};

PIDGains bedInitialGains = {-8, -5, -0.1, -12};
#define BED_INITIAL_MAX_JERK 0.004
double bedIntegralCap = 1;

PIDController bedPID(bedInitialGains, bedIntegralCap, BED_INITIAL_MAX_JERK, false, BED_ACCEPTABLE_ERROR);

enum BedState bedState;

long homeStartMillis;

double targetPosition;

void homeBed() {
    run_bed(- BED_HOME_POWER); // negative to move toward home switches
    homeStartMillis = millis();
}

void initBed() {
    bedEncoder.init(MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
    homeBed();

    addTweakableVariable("Bed Gain", [](double val){bedPID.changeGain(val);}, bedInitialGains.gain);
    addTweakableVariable("Bed P Gain", [](double val){bedPID.changePorportionalGain(val);}, bedInitialGains.porportionalGain);
    addTweakableVariable("Bed I Gain", [](double val){bedPID.changeIntegralGain(val);}, bedInitialGains.integralGain);
    addTweakableVariable("Bed D Gain", [](double val){bedPID.changeDerivativeGain(val);}, bedInitialGains.derivativeGain);
    addTweakableVariable("Bed Max Jerk", [](double val){bedPID.maxJerk = val;}, BED_INITIAL_MAX_JERK);

    addValueReadout("Bed Error", [](){return targetPosition - bedEncoder.getPosition();});
    addValueReadout("Bed Position",[](){return bedEncoder.getPosition();});
}

void activePositionBed(double targetPosition_) {
    if (bedState == HOMING) {
        return; // don't try this until it finishes homing
    }
    bedState = ACTIVE_POSITION;
    targetPosition = targetPosition_;
    bedPID.reset();
}

void tickBed() {
    // Serial.printf("Bed Position %f ", bedEncoder.getPosition());
    bedEncoder.getPosition(); // just read it in case the first read has issues

    switch (bedState) {
        double error, correctionSignal;
        case HOMING:
            if (millis() >= homeStartMillis + BED_HOME_MILLIS) {
                run_bed(0); // relax the bed
                bedState = IDLE;
                bedEncoder.zero();
            }
            break;
        case ACTIVE_POSITION:
            error = targetPosition - bedEncoder.getPosition(); 
            correctionSignal = bedPID.tick(error);
            run_bed(correctionSignal);
            break;
    }
}

void relaxBed() {
    bedState = IDLE;
    run_bed(0);
}