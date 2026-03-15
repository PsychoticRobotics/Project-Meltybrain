#include "Robot.h"
// using namespace std;

Vector3d lastAccelData;

Robot::Robot(AccelerometerManager& accelerometers, MotorManager& motors) {
    this->accelerometers = &accelerometers;
    this->motors = &motors;
}

void Robot::updateTheta(uint32_t dt) {
    Vector3d accelerometerData = accelerometers->fetchNTU();

    double accelerationNormal = accelerometerData.x();
    Serial.print("Normal: ");
    Serial.println(accelerationNormal);

    double filteredAcceleration = (0.9 * filteredAcceleration) + (0.01 * accelerationNormal);
    Serial.print("Filtered: ");
    Serial.println(filteredAcceleration);

    // double angularVelocity = sqrt(max(filteredAcceleration / accelerometerRadius, 0));
    // Serial.print("Angular velocity: ");
    // Serial.println(angularVelocity);
    // Serial.println(" rad/s");

    theta += angularVelocity * dt;
    theta = fmod(theta, 2*PI);
    // Serial.print("Theta: ");
    // Serial.print(theta);
    // Serial.println(" rad");
}

// Helper function for the move function
bool Robot::isWithinHalfTurn(double theta, double direction) {
    double diff = fmod(direction - theta + 2 * PI, 2 * PI);
    return diff < PI / 2 || diff >= 3 * PI / 2;
}

void Robot::move(float channel1, float channel2, float channel3, uint32_t dt)
{
    // Maps the received inputs from the transmitter
    channel1 = map(channel1, 994, 2014, -1, 1);
    channel2 = map(channel2, 990, 2010, -1, 1);
    channel3 = map(channel3, 1000, 2014, 0, 1);

    // Making sure there aren't any numbers that don't make sense
    if (channel1 < -1.02 || channel1 > 1.02) channel1 = 0;
    if (channel2 < -1.02 || channel2 > 1.02) channel2 = 0;
    if (channel3 < 0.05) channel3 = 0;
    if (channel3 > 1) channel3 = 1;

    // Coming up with the vector for where the bot should go
    float throttle = channel3; // How fast the bot should rotate
    float direction = atan2(channel1 / channel2);
    if (direction < 0) direction += 2*PI;
    float magnitude = sqrt(pow(channel1, 2) + pow(channel2, 2));
    // If the transmitter sticks is outside the "circle", it is maxed out
    if (magnitude > 1.0) magnitude = 1.0;
    updateTheta(dt); // Updates robot's predicted orientation

    // Printing out everything, can be commented out
    Serial.print("throttle: ");
    Serial.println(throttle);
    Serial.print("direction: ");
    Serial.println(direction);
    Serial.print("magnitude: ");
    Serial.println(magnitude);
    Serial.print("Theta: ");
    Serial.println(theta);

    // Makes the movement code work

    if (isWithinHalfTurn(theta, direction))
    {
        motors->on(throttle - magnitude / 2, throttle + magnitude / 2);

        Serial.print("Motor 1: ");
        Serial.println(throttle - magnitude / 2);
        Serial.print("Motor 2: ");
        Serial.println(throttle + magnitude / 2);
    }
    else {
        motors->on(throttle + magnitude / 2, throttle - magnitude / 2);

        Serial.print("Motor 1: ");
        Serial.println(throttle + magnitude / 2);
        Serial.print("Motor 2: ");
        Serial.println(throttle - magnitude / 2);
    }
    //theta += channel2 * 0.000001 * PI * dt; // Rotates once a microsecond at full channel 2 input
}
