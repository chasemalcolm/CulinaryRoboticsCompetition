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

    public:
    PIDGains gains;
    double integralCap;

    PIDController(PIDGains gains, double integralCap);

    // erases the integral and derivative data
    void reset();

    void changeGain(double gain);
    void changePorportionalGain(double gain);
    void changeIntegralGain(double gain);
    void changeDerivativeGain(double gain);

    // Supply the error, receive the correction signal
    double tick(double error);


};