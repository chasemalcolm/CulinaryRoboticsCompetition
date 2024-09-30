#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "configserver.h"
#include <Arduino.h>

#define ENCODER_LEFT_PIN_A 10
#define ENCODER_LEFT_PIN_B 11

#define ENCODER_RIGHT_PIN_A 14
#define ENCODER_RIGHT_PIN_B 13

#define HAPPINESS_THRESHOLD 5

double getInitialDrivePower(double target);

void updateLeftEncoder();
Encoder leftEncoder(ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B, updateLeftEncoder);
void IRAM_ATTR updateLeftEncoder() {
  leftEncoder.update();
}

void updateRightEncoder();
Encoder rightEncoder(ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B, updateRightEncoder);
void IRAM_ATTR updateRightEncoder() {
  rightEncoder.update();
}

const PIDGains straightDriveInitialGains = {
    0.05,
    -1,
    0,
    0};

const PIDGains activePositionInitialGains = {
    0.080000,
    -2,
    -0.06,
    -30
};

// maximum motor power change per tick
#define DRIVETRAIN_ACCELERATION 0.004
// when accelerating, start at this absolute power since anything less doesn't actually move the motor
#define ACCELERATION_STARTING_SPEED 0.3

// Used for straightDrive
double drivePower;
double targetDrivePower;
int targetEncoderDifference; // stored as left - right

// Used for ActivePosition
int leftEncoderTarget;
int rightEncoderTarget;

// tweakable constant defining how close the drivetrain gets to the target position before switching to active position
int ActivePositionToggleDistance = 50;

// tweakable constant defining the maximum motor power for the drive motors when active positioning
double maxAbsPositioningPower = 0.6;

// tweakable constant for maximum integral cap
double maxIntegral = 0.6;

// initialize with placeholder gains
// Gains will be set once the loop is in use.
PIDController straightDrivePID(straightDriveInitialGains, maxIntegral);
PIDController leftPositionPID(activePositionInitialGains, maxIntegral);
PIDController rightPositionPID(activePositionInitialGains, maxIntegral);

enum DrivetrainState {
    IDLE,
    DRIVING_STRAIGHT,
    DRIVING_STRAIGHT_TO_POSITION,
    ACTIVE_POSIITONING,
    SWING_TURN_LEFT,
    SWING_TURN_RIGHT
};

enum DrivetrainState drivetrainState;

void changeActivePositionGain(double gain) {

    leftPositionPID.changeGain(gain);
    rightPositionPID.changeGain(gain);
}

void changeActivePositionPorportionalGain(double gain) {
    leftPositionPID.changePorportionalGain(gain);
    rightPositionPID.changePorportionalGain(gain);
}

void changeActivePositionIntegralGain(double gain) {
    leftPositionPID.changeIntegralGain(gain);
    rightPositionPID.changeIntegralGain(gain);
}

void changeActivePositionDerivativeGain(double gain) {
    leftPositionPID.changeDerivativeGain(gain);
    rightPositionPID.changeDerivativeGain(gain);
}

void changeStraightDriveGain(double gain) {
    straightDrivePID.changeGain(gain);
}

void changeStraightDrivePorportionalGain(double gain) {
    straightDrivePID.changePorportionalGain(gain);
}

void changeStraightDriveIntegralGain(double gain) {
    straightDrivePID.changeIntegralGain(gain);
}

void changeStraightDriveDerivativeGain(double gain) {
    straightDrivePID.changeDerivativeGain(gain);
}

void changeSwitchToActivePositioningDistance(double distance) {
    ActivePositionToggleDistance = distance;
}

void changeAbsPositionPower(double power) {
    maxAbsPositioningPower = power;
}

void changeIntegralCap(double cap) {
    leftPositionPID.integralCap = cap;
    rightPositionPID.integralCap = cap;
}

bool isDrivetrainHappy() {
    return leftPositionPID.happiness > HAPPINESS_THRESHOLD && rightPositionPID.happiness > HAPPINESS_THRESHOLD && drivetrainState == ACTIVE_POSIITONING && leftPositionPID.happy && rightPositionPID.happy;
}

void initEncodedDrivetrain() {
    leftEncoder.begin();
    rightEncoder.begin();

    drivetrainState = IDLE;

    addTweakableVariable("Active Posiiton Gain", changeActivePositionGain, activePositionInitialGains.gain);
    addTweakableVariable("Active Posiiton Porportional Gain", changeActivePositionPorportionalGain, activePositionInitialGains.porportionalGain);
    addTweakableVariable("Active Posiiton Integral Gain", changeActivePositionIntegralGain, activePositionInitialGains.integralGain);
    addTweakableVariable("Active Posiiton Derivative Gain", changeActivePositionDerivativeGain, activePositionInitialGains.derivativeGain);

    addTweakableVariable("Straight Drive Gain", changeStraightDriveGain, straightDriveInitialGains.gain);
    addTweakableVariable("Straight Drive Porportional Gain", changeStraightDrivePorportionalGain, straightDriveInitialGains.porportionalGain);
    addTweakableVariable("Straight Drive Integral Gain", changeStraightDriveIntegralGain, straightDriveInitialGains.integralGain);
    addTweakableVariable("Straight Drive Derivative Gain", changeStraightDriveDerivativeGain, straightDriveInitialGains.derivativeGain);

    addTweakableVariable("Distance to swap to active positioning", changeSwitchToActivePositioningDistance, ActivePositionToggleDistance);
    addTweakableVariable("Max Abs positioning Power", changeAbsPositionPower, maxAbsPositioningPower);
    addTweakableVariable("Integral Cap", changeIntegralCap, maxIntegral);

    addValueReadout("leftError", [](){ return (double) leftEncoderTarget - leftEncoder.position;});

    addValueReadout("Left encoder", [](){return (double) leftEncoder.position;});
    addValueReadout("Right encoder", [](){return (double) rightEncoder.position;});
}

void zeroDrivetrainEncoders() {
    leftEncoder.position = 0;
    rightEncoder.position = 0;
}

int getLeftEncoderPosition() {
    return leftEncoder.position;
}

int getRightEncoderPosition() {
    return rightEncoder.position;
}

// Uses PID to avoid one motor spinning faster than the other
// power in in range -1 - 1 but use something smaller in magnitude than 1 for best results.
void driveStraight(double power) {
    straightDrivePID.reset();
    drivetrainState = DRIVING_STRAIGHT;
    targetDrivePower = power;
    drivePower = getInitialDrivePower(targetDrivePower);
    targetEncoderDifference = leftEncoder.position - rightEncoder.position;
}

void activePosition(int relativePositionLeft, int relativePositionRight) {
    drivetrainState = ACTIVE_POSIITONING;

    leftPositionPID.reset();
    rightPositionPID.reset();
    leftEncoderTarget = leftEncoder.position + relativePositionLeft;
    rightEncoderTarget = rightEncoder.position + relativePositionRight;
}

void driveStraightToPosition(int relativePosition, double power) {
    driveStraight(power * relativePosition / abs(relativePosition));

    drivetrainState = DRIVING_STRAIGHT_TO_POSITION;

    leftEncoderTarget = leftEncoder.position + relativePosition;
    rightEncoderTarget = rightEncoder.position + relativePosition;


}

void swingTurnLeft(int relativePosition, double power) {
    drivetrainState = SWING_TURN_LEFT;
    
    leftEncoderTarget = leftEncoder.position;
    rightEncoderTarget = rightEncoder.position + relativePosition;
    targetDrivePower = power * relativePosition / abs(relativePosition);
    drivePower = getInitialDrivePower(targetDrivePower);;
    leftPositionPID.reset();
}

void swingTurnRight(int relativePosition, double power) {
    drivetrainState = SWING_TURN_RIGHT;

    rightEncoderTarget = rightEncoder.position;
    leftEncoderTarget = leftEncoder.position + relativePosition;
    targetDrivePower = power * relativePosition / abs(relativePosition); // invert power if going backwards
    drivePower = getInitialDrivePower(targetDrivePower);
    rightPositionPID.reset();
}

void relaxDrivetrain() {
    drivetrainState = IDLE;
    run_drive_motors(0, 0);
}

double getInitialDrivePower(double target) {
    return constrain(target, -ACCELERATION_STARTING_SPEED, ACCELERATION_STARTING_SPEED);
}

void tickDrivingStraight(){
    int error = leftEncoder.position - rightEncoder.position - targetEncoderDifference;
    double correctionSignal = straightDrivePID.tick(error);

    steer_drivetrain(drivePower, correctionSignal);
}

void tickDrivetrain() {
    drivePower = constrain(targetDrivePower, drivePower - DRIVETRAIN_ACCELERATION, drivePower + DRIVETRAIN_ACCELERATION);
    //Serial.printf("right drive happy: %d ", rightPositionPID.happy);

    // PIDGains gains;
    switch (drivetrainState)
    {
        int leftError, rightError, averageEncoderRemainingDistance;
        double leftCorrectionSignal, rightCorrectionSignal;
        case DRIVING_STRAIGHT:
            tickDrivingStraight();
            break;

        case ACTIVE_POSIITONING:
            leftError = leftEncoder.position - leftEncoderTarget;
            rightError = rightEncoder.position - rightEncoderTarget;
            leftCorrectionSignal = leftPositionPID.tick(leftError, true); // logging
            leftCorrectionSignal = constrain(leftCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);
            rightCorrectionSignal = rightPositionPID.tick(rightError, true); //logging
            rightCorrectionSignal = constrain(rightCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);

            run_drive_motors(leftCorrectionSignal, rightCorrectionSignal);

            // gains = leftPositionPID.gains;

            // Serial.printf("LeftError %d, rightError %d ", leftError, rightError);

            // Serial.printf("OPID: %f %f %f %f \n", gains.gain, gains.porportionalGain, gains.integralGain, gains.derivativeGain);
            break;

        case DRIVING_STRAIGHT_TO_POSITION:
            averageEncoderRemainingDistance = ((leftEncoderTarget - leftEncoder.position) + (rightEncoderTarget - rightEncoder.position)) / 2;
            Serial.printf("Avg remaining dist %d ", averageEncoderRemainingDistance);
            if (abs(averageEncoderRemainingDistance) < ActivePositionToggleDistance) {
                // switch to active positioning
                drivetrainState = ACTIVE_POSIITONING;

                leftPositionPID.reset();
                rightPositionPID.reset();
            }
            else {
                tickDrivingStraight();
            }
            break;

        case SWING_TURN_LEFT:
            averageEncoderRemainingDistance = abs(rightEncoderTarget - rightEncoder.position);
            Serial.printf("Avg right remaining dist %d ", averageEncoderRemainingDistance);
            if (averageEncoderRemainingDistance < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSIITONING;

                rightPositionPID.reset();
            }
            leftError = leftEncoder.position - leftEncoderTarget;
            leftCorrectionSignal = leftPositionPID.tick(leftError);
            leftCorrectionSignal = constrain(leftCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);
            
            run_drive_motors(leftCorrectionSignal, drivePower);

            break;

        case SWING_TURN_RIGHT:
            averageEncoderRemainingDistance = abs(leftEncoderTarget - leftEncoder.position);
            Serial.printf("Avg left remaining dist %d ", averageEncoderRemainingDistance);
            if (averageEncoderRemainingDistance < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSIITONING;

                leftPositionPID.reset();
            }

            rightError = rightEncoder.position - rightEncoderTarget;
            rightCorrectionSignal = rightPositionPID.tick(rightError);
            rightCorrectionSignal = constrain(rightCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);
            
            run_drive_motors(drivePower, rightCorrectionSignal);
            
    }

    //Serial.printf("Left %d, Right %d ", leftEncoder.position, rightEncoder.position);
}


