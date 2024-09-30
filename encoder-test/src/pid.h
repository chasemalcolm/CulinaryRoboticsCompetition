#pragma once

struct PIDGains {
    double gain;
    double porportionalGain;
    double integralGain;
    double derivativeGain;
};

class PIDController {
    private:
    double integral;
    double lastError;
    double lastNonDerivativeCorrectionSignal;

    public: // you can change these if u want
    PIDGains gains;
    double integralCap;
    double maxJerk;
    bool deleteIntegral;
    double acceptableError;

    PIDController(PIDGains gains, double integralCap, double maxJerk=10000, bool deleteIntegral = true, double acceptableError = 0);

    // erases the integral and derivative data
    void reset();

    void changeGain(double gain);
    void changePorportionalGain(double gain);
    void changeIntegralGain(double gain);
    void changeDerivativeGain(double gain);

    // Supply the error, receive the correction signal
    double tick(double error);


};