#ifndef MELTYBRAIN_MOTOR_H
#define MELTYBRAIN_MOTOR_H

#include <stdint.h>

class Motor {
public:
    void init(int index);       // 0 = left, 1 = right
    void on(float throttle);    // -1.0 to 1.0
    void off();
    void coast();

private:
    int motorIndex;
};

class MotorManager {
public:
    void init();                                        // arm both ESCs
    void on(float throttle);                            // both motors, -1.0 to 1.0
    void on(float throttle1, float throttle2);          // individual, -1.0 to 1.0
    void off(int motor = 0);                            // 0 = both, 1 = motor1, 2 = motor2
    void coast(int motor = 0);

    bool getRPM(int motor, int16_t &rpm);               // motor 1 or 2
    bool getVoltage(int motor, float &volts);
    bool getCurrent(int motor, float &amps);
    bool getTemp(int motor, uint8_t &degC);

private:
    Motor motor1;
    Motor motor2;
};

#endif //MELTYBRAIN_MOTOR_H
