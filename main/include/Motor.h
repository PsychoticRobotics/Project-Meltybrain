#ifndef MELTYBRAIN_MOTOR_H
#define MELTYBRAIN_MOTOR_H

#include <Servo.h>

class Motor {
public:
    void init(int pin);
    void on(float throttle_percent);
    void off();
    void coast();

private:
    Servo base;
    int pwm_base = 1000;
    int pwm_range = 1000;
};

class MotorManager {
public:

    //initialize motors
    void init(int pin1, int pin2);

    //turn motor_X_on
    void on(float motor_percent);
    void on(float motor_percent1, float motor_percent2);

    //motors shut-down (robot not translating)
    void off(int motor = 0); //0 = both, 1 = motor 1, 2 = motor 2

    void coast(int motor = 0); //0 = both, 1 = motor 1, 2 = motor 2

private:
    Motor motor1;
    Motor motor2;
};

#endif //MELTYBRAIN_MOTOR_H