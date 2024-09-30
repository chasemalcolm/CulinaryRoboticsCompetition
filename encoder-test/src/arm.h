#pragma once

// Init arm before bed
// Assumes that Wire is already init'ed
void initArm();

// call this at a consistent interval for consistent PID behaviour.
void tickArm();

// rotates arm to the desired position
void activePositionArm(double targetPosition_);

void extendArm(double power);

void retractArm(double power);

void relaxArmRotation();

// Returns true only if arm is not extending
bool isArmExtensionIdle();