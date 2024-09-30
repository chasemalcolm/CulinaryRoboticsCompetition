#include "pid.h"
#include <Arduino.h>

#define ACCEPTABLE_ERROR 0.0001

PIDController::PIDController(PIDGains gains_, double integralCap_) {
    gains = gains_;
    integralCap = integralCap_;
} 

void PIDController::reset() {
    integral = 0;
    lastError = 0;
}

void PIDController::changeGain(double gain) {
    gains.gain = gain;
    reset();
}

void PIDController::changePorportionalGain(double gain) {
    gains.porportionalGain = gain;
    reset();
}

void PIDController::changeIntegralGain(double gain) {
    gains.integralGain = gain;
    reset();
}

void PIDController::changeDerivativeGain(double gain) {
    gains.derivativeGain = gain;
    reset();
}

double PIDController::tick(double error) {
    integral += error;
    double derivative = error - lastError;

    Serial.printf("D: %f Error: %f ",derivative, error);

    lastError = error;

    if (error == 0 && derivative == 0) {
        integral = 0;
    }

    /*Serial.printf("PID Error %f ", error);

    Serial.printf("PID Gains %f %f %f %f ", gains.gain, gains.porportionalGain, gains.derivativeGain, gains.integralGain);*/
    double integralContribution = gains.gain * gains.integralGain * integral;
    integralContribution = constrain(integralContribution, -integralCap, integralCap);

    return gains.gain * (gains.porportionalGain * error + gains.derivativeGain * derivative) + integralContribution;
}

