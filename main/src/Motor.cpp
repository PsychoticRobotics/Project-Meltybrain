#include "Motor.h"

void Motor::init(int pin) {
    base.attach(pin);
    base.writeMicroseconds(PWM_BASE);
}

void Motor::on(float throttle_percent) {
    base.writeMicroseconds(PWM_BASE + (throttle_percent / 100.0f) * PWM_RANGE);
}

void Motor::off() {
    base.writeMicroseconds(PWM_BASE);
}

void Motor::coast() {
    base.writeMicroseconds(PWM_BASE);
}

void MotorManager::init(int pin1, int pin2) {
    motor1.init(pin1);
    motor2.init(pin2);
}

void MotorManager::on(float throttle_percent) {
    motor1.on(throttle_percent);
    motor2.on(throttle_percent);
}

void MotorManager::on(float throttle_percent1, float throttle_percent2) {
    motor1.on(throttle_percent1);
    motor2.on(throttle_percent2);
}

void MotorManager::off(int motor) {
    if (motor == 1)
        motor1.off();
    else if (motor == 2)
        motor2.off();
    else {
        motor1.off();
        motor2.off();
    }
}

void MotorManager::coast(int motor) {
    if (motor == 1)
        motor1.coast();
    else if (motor == 2)
        motor2.coast();
    else {
        motor1.coast();
        motor2.coast();
    }
}
