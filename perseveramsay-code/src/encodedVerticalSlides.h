#pragma once

void initVerticalSlides();

// call this at a consistent interval for consistent PID behaviour.
void tickVerticalSlides();

void activePositionVerticalSlides(double targetPosition_);

bool vertical_slide_to_integral_cut();