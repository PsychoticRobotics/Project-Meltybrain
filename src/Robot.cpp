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
    channel1 = map(channel1, 994, 2014, -1, 1);
    channel2 = map(channel2, 990, 2010, -1, 1);
    channel3 = map(channel3, 1000, 2014,  0, 1);

    if (channel1 < -1.02 || channel1 > 1.02) channel1 = 0;
    if (channel2 < -1.02 || channel2 > 1.02) channel2 = 0;
    if (channel3 < 0.05) channel3 = 0;
    if (channel3 > 1)    channel3 = 1;

    float throttle  = channel3;
    float direction = atan2(channel1, channel2);
    if (direction < 0) direction += 2 * PI;
    float magnitude = sqrt(pow(channel1, 2) + pow(channel2, 2));
    if (magnitude > 1.0) magnitude = 1.0;

    updateTheta();  // reads latest fused angle from AngleEstimator

    if (isWithinHalfTurn(theta, direction)) {
        _motors->on(throttle - magnitude / 2, throttle + magnitude / 2);
    } else {
        _motors->on(throttle + magnitude / 2, throttle - magnitude / 2);
    }
}
