#include "Robot.h"

Robot::Robot(AngleEstimator& estimator, MotorManager& motors)
    : _estimator(&estimator), _motors(&motors) {}

void Robot::updateTheta() {
    theta = _estimator->getAngle();
}

bool Robot::isWithinHalfTurn(double theta, double direction) {
    double diff = fmod(direction - theta + 2 * PI, 2 * PI);
    return diff < PI / 2 || diff >= 3 * PI / 2;
}

void Robot::move(float channel1, float channel2, float channel3) {
    // Map receiver inputs to [-1, 1] / [0, 1]
    channel1 = (channel1 - 994.0f)  / (2014.0f - 994.0f)  * 2.0f - 1.0f;
    channel2 = (channel2 - 990.0f)  / (2010.0f - 990.0f)  * 2.0f - 1.0f;
    channel3 = (channel3 - 1000.0f) / (2014.0f - 1000.0f);

    if (channel1 < -1.02 || channel1 > 1.02) channel1 = 0;
    if (channel2 < -1.02 || channel2 > 1.02) channel2 = 0;
    if (channel3 < 0.05) channel3 = 0;
    if (channel3 > 1)    channel3 = 1;

    float throttle  = channel3;
    float direction = atan2(channel1, channel2);
    if (direction < 0) direction += 2 * PI;
    float magnitude = sqrtf(channel1 * channel1 + channel2 * channel2);
    if (magnitude > 1.0f) magnitude = 1.0f;
    // Clamp magnitude so average stays exactly at throttle and both motors stay in [0, 1].
    magnitude = fminf(magnitude, fminf(2.0f * throttle, 2.0f * (1.0f - throttle)));

    float lo = throttle - magnitude / 2.0f;
    float hi = throttle + magnitude / 2.0f;

    updateTheta();  // reads latest fused angle from AngleEstimator

    bool inHalf = isWithinHalfTurn(theta, direction);
    if (spinReversed) inHalf = !inHalf;
    if (inHalf) {
        _motors->on(lo, hi);
    } else {
        _motors->on(hi, lo);
    }
}
