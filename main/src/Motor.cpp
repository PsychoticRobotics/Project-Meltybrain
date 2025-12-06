#include "Motor.h"

void Motor::init(int pin) {
    base.attach(pin);
    base.writeMicroseconds(pwm_base);
}

void Motor::on(float throttle_percent) {
    base.writeMicroseconds(pwm_base + (throttle_percent / 100.0f) * pwm_range);
}

void Motor::off() {
    base.writeMicroseconds(pwm_base);
}

void Motor::coast() {
    base.writeMicroseconds(pwm_base);
}

void MotorManager::init(int pin1, int pin2) {
    motor1.init(pin1);
    motor2.init(pin2);
}

void MotorManager::on(float motor_percent) {
    motor1.on(motor_percent);
    motor2.on(motor_percent);
}

void MotorManager::on(float motor_percent1, float motor_percent2) {
    motor1.on(motor_percent1);
    motor2.on(motor_percent2);
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
