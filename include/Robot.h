#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "AngleEstimator.h"
#include "Motor.h"
#include <Arduino.h>

class Robot {
public:
    Robot(AngleEstimator& estimator, MotorManager& motors);

    void move(float channel1, float channel2, float channel3);
    bool isWithinHalfTurn(double theta, double direction);

    double theta = 0.0f;  // robot's orientation angle in radians

private:
    void updateTheta();   // reads latest angle from estimator

    AngleEstimator* _estimator;
    MotorManager*   _motors;
};

#endif
