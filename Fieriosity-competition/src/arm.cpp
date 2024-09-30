#include "arm.h"
#include <Arduino.h>
#include "motor_control.h"
#include "pid.h"
#include "configServer.h"
#include <I2CEncoder.h>
#include "encoder.h"

#define ARM_HOME_POWER 0.4


#define ARM_HOME_MILLIS 2500

#define ARM_EXTENSION_ENCODER_PIN_A 9
#define ARM_EXTENSION_ENCODER_PIN_B 12

#define ARM_EXTENSION_MAX_PIN 40
#define ARM_EXTENSION_MIN_PIN 41

#define ARM_ACCEPTABLE_ERROR 0.02

// The arm will be considered "done extending" if extending or retraction for more than this time
#define ARM_EXTENSION_TIMEOUT_MILLIS 3000

I2CEncoder armEncoder;

void updateExtensionEncoder();
Encoder extensionEncoder(ARM_EXTENSION_ENCODER_PIN_A, ARM_EXTENSION_ENCODER_PIN_B, updateExtensionEncoder);
void IRAM_ATTR updateExtensionEncoder() {
  extensionEncoder.update();
}

int getExtensionPosition() {
    return extensionEncoder.position;
}

enum ArmExtensionState {
    EXTENDING,
    RETRACTING,
    RETRACTION_BRAKING,
    NOT_EXTENDING,
    RETRACT_TO_POSITION
};

enum ArmExtensionState armExtensionState;

enum ArmRotationState {
    HOMING,
    ACTIVE_POSITION,
    IDLE
};

enum ArmRotationState armRotationState;

PIDGains armInitialGains = {-8, -2.5, -0.05, -16};
#define ARM_INITIAL_MAX_JERK 0.01
double armIntegralCap = 1;

PIDController armPID(armInitialGains, armIntegralCap, ARM_INITIAL_MAX_JERK, false, ARM_ACCEPTABLE_ERROR);

long armHomeStartMillis;

int extensionTargetPosition;
double armTargetPosition;
double retractionPower;

long retractionBrakingStartMillis;
// tweakable constant for how long to brake at the end of retraction
// braking happens at the same power at retraction
long retractionBrakingTime_ms = 50;

long extensionStartMillis;

void homeArmRotation() {
    armRotationState = HOMING;
    run_arm_rotation(ARM_HOME_POWER); // negative to move toward home switches
    armHomeStartMillis = millis();
}

void initArm() {
    pinMode(ARM_EXTENSION_MAX_PIN, INPUT_PULLUP);
    pinMode(ARM_EXTENSION_MIN_PIN, INPUT_PULLUP);
    extensionEncoder.begin();
    armEncoder.init(MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
    homeArmRotation();
    armExtensionState = NOT_EXTENDING;

    // delay(1000); // maybe this helps the second encoder init?

    addTweakableVariable("Arm Gain", [](double val){armPID.changeGain(val);}, armInitialGains.gain);
    addTweakableVariable("Arm P Gain", [](double val){armPID.changePorportionalGain(val);}, armInitialGains.porportionalGain);
    addTweakableVariable("Arm I Gain", [](double val){armPID.changeIntegralGain(val);}, armInitialGains.integralGain);
    addTweakableVariable("Arm D Gain", [](double val){armPID.changeDerivativeGain(val);}, armInitialGains.derivativeGain);
    addTweakableVariable("Arm Max Jerk", [](double val){armPID.maxJerk = val;}, ARM_INITIAL_MAX_JERK);
    addTweakableVariable("Arm retraction Time", [](double brakingTime){retractionBrakingTime_ms = brakingTime;}, retractionBrakingTime_ms);
    addTweakableVariable("Retract Arm to position (0.9 power)", [](double pos){positionExtension(0.9, pos);}, 0);

    addValueReadout("Arm Error", [](){return armTargetPosition - armEncoder.getPosition();});
    addValueReadout("Arm Position",[](){return armEncoder.getPosition();});
    addValueReadout("Arm state", [](){return (double) armRotationState;});
    addValueReadout("Arm Extension max", [](){return (double) digitalRead(ARM_EXTENSION_MAX_PIN);});
    addValueReadout("Arm Extension min", [](){return (double) digitalRead(ARM_EXTENSION_MIN_PIN);});
    addValueReadout("Arm Extension Encoder Value", [](){return (double) extensionEncoder.position;});
}

void activePositionArm(double armTargetPosition_) {
    if (armRotationState == HOMING) {
        return; // don't try this until it finishes homing
    }
    armRotationState = ACTIVE_POSITION;
    armTargetPosition = armTargetPosition_;
    //armPID.reset(); we went to keep the integral for smooth positioning
}

void positionExtension(double power, int targetPosition) {
    retractArm(power);
    armExtensionState = RETRACT_TO_POSITION;
    extensionTargetPosition = targetPosition;
}

void extendArm(double power) {
    armExtensionState = EXTENDING;
    extensionStartMillis = millis();
    run_arm_extension(power);
}

void retractArm(double power) {
    armExtensionState = RETRACTING;
    retractionPower = power;
    extensionStartMillis = millis();
    run_arm_extension(-power);
}

void relaxArmRotation() {
    armRotationState = IDLE;
    run_arm_rotation(0);
}

void brakeAtEndOfRetraction() {
    armExtensionState = RETRACTION_BRAKING;
    retractionBrakingStartMillis = millis();
    run_arm_extension(retractionPower);
}

void stopArmExtension() {
    brakeAtEndOfRetraction();
}

void tickArm() {
    // Serial.printf("Arm Position %f ", armEncoder.getPosition());

    switch (armRotationState) {
        double error, correctionSignal;
        case HOMING:
            if (millis() >= armHomeStartMillis + ARM_HOME_MILLIS) {
                run_arm_rotation(0); // relax the arm
                armRotationState = IDLE;
                armEncoder.zero();
            }
            break;
        case ACTIVE_POSITION:
            error = armTargetPosition - armEncoder.getPosition(); 
            correctionSignal = armPID.tick(error);
            run_arm_rotation(correctionSignal);
            break;
    }

    switch (armExtensionState) {
        case EXTENDING:
            if (!digitalRead(ARM_EXTENSION_MAX_PIN) || millis() > extensionStartMillis + ARM_EXTENSION_TIMEOUT_MILLIS) {
                armExtensionState = NOT_EXTENDING;
                run_arm_extension(0);
                extensionEncoder.position = 0;
            }
            break;
        case RETRACT_TO_POSITION:
            if (extensionEncoder.position > extensionTargetPosition || !digitalRead(ARM_EXTENSION_MIN_PIN) || millis() > extensionStartMillis + ARM_EXTENSION_TIMEOUT_MILLIS) {
                brakeAtEndOfRetraction();
            }
            break;
        case RETRACTING:
            if (!digitalRead(ARM_EXTENSION_MIN_PIN) || millis() > extensionStartMillis + ARM_EXTENSION_TIMEOUT_MILLIS) {
                brakeAtEndOfRetraction();
            }
            break;
        case RETRACTION_BRAKING:
            if (millis() > retractionBrakingStartMillis + retractionBrakingTime_ms) {
                armExtensionState = NOT_EXTENDING;
                run_arm_extension(0);
            }
    }
}

bool isArmExtensionIdle() {
    return armExtensionState == NOT_EXTENDING;
}


