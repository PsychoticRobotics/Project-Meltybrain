#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "Accelerometer.h"
#include "Motor.h"
#include <Arduino.h>

class Robot {
public:
    Robot(AccelerometerManager& accelerometers, MotorManager& motors);

    void updateTheta(uint32_t dt);  // Updates the robot's orientation (theta) based on accelerometer data
    void move(float channel1, float channel2, float channel3, uint32_t dt);

private:
    double theta = 0.0f; // Robot's orientation angle in radians
    double accelerometerRadius = 0.033; // meters
    double accelerometerInterval = 0.000001; // seconds
    double filteredAcceleration = 0.0f;
    AccelerometerManager *accelerometers;

    MotorManager *motors;
};

#endif