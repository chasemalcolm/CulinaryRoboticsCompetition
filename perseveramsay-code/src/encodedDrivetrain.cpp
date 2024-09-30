#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "configserver.h"
#include "encodedDrivetrain.h"
#include "math.h"
#include <Arduino.h>
#include "tape_following.h"

#define ENCODER_LEFT_PIN_A 9
#define ENCODER_LEFT_PIN_B 10

#define ENCODER_RIGHT_PIN_A 11
#define ENCODER_RIGHT_PIN_B 12

bool driving_for_set_distance;
bool swing_turning_for_set_arc;
bool on_spot_turning_for_set_arc;

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

const PIDGains straightDriveInitialGains = {-1, 0.04, 0, 0.05};

const PIDGains activePositionInitialGains = {-0.08, 2, 0.13, 9};

// tweakable constant for maximum integral cap
double maxIntegral = 0.625;


// Used for straightDrive
double drivePower;
int targetEncoderDifference; // stored as left - right

// Used for ActivePosition
int leftEncoderTarget;
int rightEncoderTarget;

// tweakable constant defining how close the drivetrain gets to the target position before switching to active position
int ActivePositionToggleDistance = 50;

// tweakable constant defining the maximum motor power for the drive motors when active positioning
double maxAbsPositioningPower = 0.65;

// initialize with placeholder gains
// Gains will be set once the loop is in use.
PIDController straightDrivePID(straightDriveInitialGains, maxIntegral, 10000, true);
PIDController leftPositionPID(activePositionInitialGains, maxIntegral, 10000, true);
PIDController rightPositionPID(activePositionInitialGains, maxIntegral, 10000, true);

enum DrivetrainState {
    IDLE,
    DRIVING_STRAIGHT,
    DRIVING_STRAIGHT_TO_POSITION,
    ACTIVE_POSITIONING,
    SWING_TURN_LEFT,
    SWING_TURN_RIGHT,
    ON_SPOT_TURN_LEFT,
    ON_SPOT_TURN_RIGHT
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

void initEncodedDrivetrain() {
    leftEncoder.begin();
    rightEncoder.begin();

    drivetrainState = IDLE;
    driving_for_set_distance = false;
    swing_turning_for_set_arc = false;
    on_spot_turning_for_set_arc = false;

    addTweakableVariable("Active Position Gain", changeActivePositionGain, activePositionInitialGains.gain);
    addTweakableVariable("Active Position Porportional Gain", changeActivePositionPorportionalGain, activePositionInitialGains.porportionalGain);
    addTweakableVariable("Active Position Integral Gain", changeActivePositionIntegralGain, activePositionInitialGains.integralGain);
    addTweakableVariable("Active Position Derivative Gain", changeActivePositionDerivativeGain, activePositionInitialGains.derivativeGain);

    addTweakableVariable("Straight Drive Gain", changeStraightDriveGain, straightDriveInitialGains.gain);
    addTweakableVariable("Straight Drive Porportional Gain", changeStraightDrivePorportionalGain, straightDriveInitialGains.porportionalGain);
    addTweakableVariable("Straight Drive Integral Gain", changeStraightDriveIntegralGain, straightDriveInitialGains.integralGain);
    addTweakableVariable("Straight Drive Derivative Gain", changeStraightDriveDerivativeGain, straightDriveInitialGains.derivativeGain);

    addTweakableVariable("Distance to swap to active positioning", changeSwitchToActivePositioningDistance, ActivePositionToggleDistance);
    addTweakableVariable("Max Abs positioning Power", changeAbsPositionPower, maxAbsPositioningPower);
    addTweakableVariable("Integral Cap", changeIntegralCap, maxIntegral);

    addValueReadout("leftError", [](){ return (double) leftEncoderTarget - leftEncoder.position;});
    addValueReadout("rightError", [](){ return (double) rightEncoderTarget - rightEncoder.position;});
}

// Uses PID to avoid one motor spinning faster than the other
// power in in range -1 - 1 but use something smaller in magnitude than 1 for best results.
void driveStraight(double power) {
    straightDrivePID.reset();
    drivetrainState = DRIVING_STRAIGHT;
    drivePower = power;
    targetEncoderDifference = leftEncoder.position - rightEncoder.position;
}

void activePosition(int relativePositionLeft, int relativePositionRight) {
    drivetrainState = ACTIVE_POSITIONING;

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
    drivePower = power * relativePosition / abs(relativePosition);
    leftPositionPID.reset();
    rightPositionPID.reset();
}

void swingTurnRight(int relativePosition, double power) {
    drivetrainState = SWING_TURN_RIGHT;

    rightEncoderTarget = rightEncoder.position;
    leftEncoderTarget = leftEncoder.position + relativePosition;
    drivePower = power * relativePosition / abs(relativePosition); // invert power if going backwards
    rightPositionPID.reset();
    leftPositionPID.reset();
}

void turnLeftOnSpot(int relativePosition, double power) {
    drivetrainState = ON_SPOT_TURN_LEFT;
    
    leftEncoderTarget = leftEncoder.position - relativePosition;
    rightEncoderTarget = rightEncoder.position + relativePosition;
    drivePower = power * relativePosition / abs(relativePosition);
    leftPositionPID.reset();
    rightPositionPID.reset();
}

void turnRightOnSpot(int relativePosition, double power) {
    drivetrainState = ON_SPOT_TURN_RIGHT;
    
    leftEncoderTarget = leftEncoder.position + relativePosition;
    rightEncoderTarget = rightEncoder.position - relativePosition;
    drivePower = power * relativePosition / abs(relativePosition);
    leftPositionPID.reset();
    rightPositionPID.reset();
}

void relaxDrivetrain() {
    drivetrainState = IDLE;
    run_drive_motors(0, 0);
}

bool drive_for_set_distance(int relative_position) {

    if(!driving_for_set_distance){
        leftEncoderTarget = leftEncoder.position + relative_position;
        rightEncoderTarget = rightEncoder.position + relative_position;
        driving_for_set_distance = true;
    }

    if((abs(leftEncoderTarget - leftEncoder.position + rightEncoderTarget - rightEncoder.position)/2 < ActivePositionToggleDistance) 
        && driving_for_set_distance && drivetrainState != ACTIVE_POSITIONING){

        stop_following_main_line();
        drivetrainState = ACTIVE_POSITIONING;
        leftPositionPID.reset();
        rightPositionPID.reset();
    }

    if (drive_to_integral_cut()) {
        driving_for_set_distance = false;
        return true;
    }

    return false;
}

bool drive_for_set_distance(int relative_position, int toggleDistance) {

    if(!driving_for_set_distance){
        leftEncoderTarget = leftEncoder.position + relative_position;
        rightEncoderTarget = rightEncoder.position + relative_position;
        driving_for_set_distance = true;
    }

    if((abs(leftEncoderTarget - leftEncoder.position + rightEncoderTarget - rightEncoder.position)/2 < toggleDistance) 
        && driving_for_set_distance && drivetrainState != ACTIVE_POSITIONING){

        stop_following_main_line();
        drivetrainState = ACTIVE_POSITIONING;
        leftPositionPID.reset();
        rightPositionPID.reset();
    }

    if (drive_to_integral_cut()) {
        driving_for_set_distance = false;
        return true;
    }

    return false;
}

bool drive_to_integral_cut() {

    if (drivetrainState == ACTIVE_POSITIONING 
        && ((leftPositionPID.integralDeleted 
        && rightPositionPID.integralDeleted))) {
        return true;
    }

    return false;
}

void tickDrivingStraight(){
    int error = leftEncoder.position - rightEncoder.position - targetEncoderDifference;
    double correctionSignal = straightDrivePID.tick(error);

    steer_drivetrain(drivePower, correctionSignal);
}

void tickDrivetrain() {

    PIDGains gains;
    switch (drivetrainState)
    {
        int leftError, rightError, averageEncoderRemainingDistance;
        double leftCorrectionSignal, rightCorrectionSignal;
        case DRIVING_STRAIGHT:
            tickDrivingStraight();
            break;

        case ACTIVE_POSITIONING:
            leftError = leftEncoder.position - leftEncoderTarget;
            rightError = rightEncoder.position - rightEncoderTarget;
            leftCorrectionSignal = leftPositionPID.tick(leftError);
            leftCorrectionSignal = constrain(leftCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);
            rightCorrectionSignal = rightPositionPID.tick(rightError);
            rightCorrectionSignal = constrain(rightCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);

            run_drive_motors(leftCorrectionSignal, rightCorrectionSignal);

            gains = leftPositionPID.gains;

            break;

        case DRIVING_STRAIGHT_TO_POSITION:
            averageEncoderRemainingDistance = ((leftEncoderTarget - leftEncoder.position) + (rightEncoderTarget - rightEncoder.position)) / 2;
            if (abs(averageEncoderRemainingDistance) < ActivePositionToggleDistance) {
                // switch to active positioning
                drivetrainState = ACTIVE_POSITIONING;

                leftPositionPID.reset();
                rightPositionPID.reset();
            }
            else {
                tickDrivingStraight();
            }
            break;

        case SWING_TURN_LEFT:
            if (abs(rightEncoderTarget - rightEncoder.position) < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSITIONING;

                rightPositionPID.reset();
                leftPositionPID.reset();
            }
            leftError = leftEncoder.position - leftEncoderTarget;
            leftCorrectionSignal = leftPositionPID.tick(leftError);
            leftCorrectionSignal = constrain(leftCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);
            
            run_drive_motors(leftCorrectionSignal, drivePower);

            break;

        case SWING_TURN_RIGHT:

            if (abs(leftEncoderTarget - leftEncoder.position) < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSITIONING;

                leftPositionPID.reset();
                rightPositionPID.reset();
            }

            rightError = rightEncoder.position - rightEncoderTarget;
            rightCorrectionSignal = rightPositionPID.tick(rightError);
            rightCorrectionSignal = constrain(rightCorrectionSignal, -maxAbsPositioningPower, maxAbsPositioningPower);
            
            run_drive_motors(drivePower, rightCorrectionSignal);
            
            break;
            
        case ON_SPOT_TURN_LEFT:
            if (abs(rightEncoderTarget - rightEncoder.position) < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSITIONING;

                rightPositionPID.reset();
                leftPositionPID.reset();
            }
            
            if (abs(leftEncoderTarget - leftEncoder.position) < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSITIONING;

                leftPositionPID.reset();
                rightPositionPID.reset();
            }

            run_drive_motors(-drivePower, drivePower);

            break;

        case ON_SPOT_TURN_RIGHT:
            if (abs(rightEncoderTarget - rightEncoder.position) < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSITIONING;

                rightPositionPID.reset();
                leftPositionPID.reset();
            }
            
            if (abs(leftEncoderTarget - leftEncoder.position) < ActivePositionToggleDistance) {
                drivetrainState = ACTIVE_POSITIONING;

                leftPositionPID.reset();
                rightPositionPID.reset();
            }

            run_drive_motors(drivePower, -drivePower);

            break;
    }

}


