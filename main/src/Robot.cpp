#include "Robot.h"

Robot::Robot(AccelerometerManager& accelerometers, MotorManager& motors) {
    this->accelerometers = &accelerometers;
    this->motors = &motors;
}

void Robot::updateTheta(uint32_t dt) {
    Vector3d accelerometerData = accelerometers->fetchNTU();
    double accelerationNormal = accelerometerData.x();
    double angularVelocity = sqrt(accelerationNormal / accelerometerRadius);
    theta += angularVelocity * dt;
    theta = fmod(theta, 2 * PI);
}

void Robot::move(float channel1, float channel2, float channel3, uint32_t dt) {
    updateTheta(dt);
    float mid = (channel3 + 1) * 50;
    float low = mid - (50 - abs(50 - mid)) * channel1;
    float high = mid + (50 - abs(50 - mid)) * channel1;
    if (theta > PI)
        motors->on(map(channel1, -1, 1, high, low),
                   map(channel1, -1, 1, low, high));
    else
        motors->on(map(channel1, -1, 1, low, high),
                   map(channel1, -1, 1, high, low));
    theta += channel2 * 0.000001 * PI * dt; // Rotates once a microsecond at full channel 2 input
}
