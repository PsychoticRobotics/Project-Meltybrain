#ifndef MELTYBRAIN_MOTOR_H
#define MELTYBRAIN_MOTOR_H

#include <Servo.h>

#define PWM_BASE 1000
#define PWM_RANGE 1000

class Motor {
public:
    void init(int pin);
    void on(float throttle_percent);
    void off();
    void coast();

private:
    Servo base;
};

class MotorManager {
public:

    //initialize motors
    void init(int pin1, int pin2);

    //turn motor_X_on
    void on(float throttle_percent);
    void on(float throttle_percen1, float throttle_percent2);

    //motors shut-down (robot not translating)
    void off(int motor = 0); //0 = both, 1 = motor 1, 2 = motor 2

    void coast(int motor = 0); //0 = both, 1 = motor 1, 2 = motor 2

private:
    Motor motor1;
    Motor motor2;
};

#endif //MELTYBRAIN_MOTOR_H