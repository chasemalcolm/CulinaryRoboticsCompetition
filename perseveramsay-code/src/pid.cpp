#include "pid.h"
#include <Arduino.h>

// IntegralCap limits the maximum integral magnitude, maxJerk limitsthe maximum absolute difference between this correction signal and the previous tick's
PIDController::PIDController(PIDGains gains_, double integralCap_, double maxJerk_, bool deleteIntegral_, double acceptableError_, double derivativeSmoothingStrength_) {
    gains = gains_;
    integralCap = integralCap_;
    maxJerk = maxJerk_;
    deleteIntegral = deleteIntegral_;
    acceptableError = acceptableError_;
    integralDeleted = false;
    derivativeSmoothingStrength = derivativeSmoothingStrength_;
    hasLastError = false;
    happy = false;
    integralDeletedCount = 0;
} 

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    lastNonDerivativeCorrectionSignal = 0;
    integralDeleted = false;
    hasLastError = false;
    integralDeletedCount = 0;
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

    if (abs(error) > acceptableError) {
        integral += error;
    }

    double unSmoothedDerivative = (error - lastError);
    
    double derivative = unSmoothedDerivative * (1 - derivativeSmoothingStrength) + lastDerivative * derivativeSmoothingStrength;
    lastDerivative = derivative;

    // Serial.printf("D: %f Error: %f ",derivative, error);

    lastError = error;

    if (deleteIntegral && abs(error) <= acceptableError && unSmoothedDerivative == 0) {
        integralDeletedCount++;
        integral = 0;
    } else {
        integralDeletedCount = 0;
    }

    if (integralDeletedCount >= 10) {
        integralDeleted = true;
    } else {
        integralDeleted = false;
    }

    double max_integral_magnitude = abs(integralCap / gains.gain / gains.integralGain);
    integral = constrain(integral, -max_integral_magnitude, max_integral_magnitude);
    
    double porportionalContribution = gains.gain * gains.porportionalGain * error;
    double integralContribution = gains.gain * gains.integralGain * integral;

    double derivativeContribution = gains.gain * gains.derivativeGain * derivative;

    // Serial.printf("PC: %f, IC: %f, DC: %f ", porportionalContribution, integralContribution, derivativeContribution);

    double nonDerivativeCorrectionSignal =  porportionalContribution + integralContribution;
    nonDerivativeCorrectionSignal = constrain(nonDerivativeCorrectionSignal, min(0.0, lastNonDerivativeCorrectionSignal - maxJerk), max(0.0, lastNonDerivativeCorrectionSignal + maxJerk));
    
    double correctionSignal;
    if (hasLastError) { // if the derivative is actually valid
        correctionSignal = nonDerivativeCorrectionSignal + derivativeContribution;
    }
    else {
        correctionSignal = nonDerivativeCorrectionSignal;
        lastDerivative = 0;
    }
    // Serial.printf("CS: %f \n", correctionSignal);
    lastNonDerivativeCorrectionSignal = nonDerivativeCorrectionSignal;
    hasLastError = true;
    return correctionSignal;
}
