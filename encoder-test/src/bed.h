#pragma once

// Initbed after arm so that the I2C encoders are added in the correct order.
// Assumes that Wire is already init'ed
void initBed();

// call this at a consistent interval for consistent PID behaviour.
void tickBed();

void activePositionBed(double targetPosition_);

void relaxBed();