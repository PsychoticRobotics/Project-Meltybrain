#include "Robot.h"

Robot::Robot(AccelerometerManager& accelerometers, MotorManager& motors) {
    this->accelerometers = &accelerometers;
    this->motors = &motors;
}

void Robot::updateTheta(uint32_t dt) {
    Vector3d accelerometerData = accelerometers->fetchNTU();
    double accelerationNormal = accelerometerData.x();
    //Serial.print("Normal: ");
    //Serial.println(accelerationNormal);
    filteredAcceleration = (0.9 * filteredAcceleration) + (0.1 * accelerationNormal);
    //Serial.print("Filtered: ");
    //Serial.println(filteredAcceleration);
    double angularVelocity = sqrt(filteredAcceleration / accelerometerRadius);
    theta += angularVelocity * dt;
    theta = fmod(theta, 2 * PI);
}

void Robot::move(float channel1, float channel2, float channel3, uint32_t dt) {
    updateTheta(dt);
    channel1 = map(channel1, 994, 2014, -1, 1);
    if (channel1 < -1 || channel1 > 1) channel1 = 0;
    channel2 = map(channel2, 990, 2010, -1, 1);
    if (channel2 < -1 || channel2 > 1) channel2 = 0;
    channel3 = map(channel3, 1000, 2014, -1, 1);
    if (channel3 < -0.90) channel3 = -1;
    if (channel3 > 1) channel3 = 1;

    float mid = (channel3 + 1) * 50;
    //Serial.print("mid: ");
    //Serial.println(mid);
    float low = mid - (50 - abs(50 - mid)) * channel1;
    //Serial.print("low: ");
    //Serial.println(low);
    float high = mid + (50 - abs(50 - mid)) * channel1;
    //Serial.print("high: ");
    //Serial.println(high);
    Serial.print("Theta: ");
    Serial.println(theta);
    if (theta > PI) {
        motors->on(map(channel1, -1, 1, high, low),
                   map(channel1, -1, 1, low, high));
        //Serial.print("Motor 1: ");
        Serial.println(map(channel1, -1, 1, high, low));
        //Serial.print("Motor 2: ");
        Serial.println(map(channel1, -1, 1, low, high));
    }
    else {
        motors->on(map(channel1, -1, 1, low, high),
                   map(channel1, -1, 1, high, low));
        //Serial.print("Motor 1: ");
        Serial.println(map(channel1, -1, 1, low, high));
        //Serial.print("Motor 2: ");
        Serial.println(map(channel1, -1, 1, high, low));
    }
    theta += channel2 * 0.000001 * PI * dt; // Rotates once a second at full channel 2 input
}