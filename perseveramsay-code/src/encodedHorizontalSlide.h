#pragma once

void initHorizontalSlide();

// call this at a consistent interval for consistent PID behaviour.
void tickHorizontalSlide();

void activePositionHorizontalSlide(double targetPosition_);

bool horizontal_slide_to_integral_cut();