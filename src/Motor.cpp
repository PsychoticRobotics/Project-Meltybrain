#include "Motor.h"
#include "ESCCMD.h"
#include <Arduino.h>

// --- Motor ---

void Motor::init(int index) {
    motorIndex = index;
}

void Motor::on(float throttle) {
    throttle = constrain(throttle, -1.0f, 1.0f);
    ESCCMD_throttle(motorIndex, (int16_t)(throttle * ESCCMD_MAX_3D_THROTTLE));
}

void Motor::off() {
    ESCCMD_throttle(motorIndex, 0);
}

// --- MotorManager ---

void MotorManager::init() {
    ESCCMD_init(2);
    int r;
    r = ESCCMD_3D_on_silent();
    Serial.printf("[Motors] 3D_on_silent: %d\n", r);
    r = ESCCMD_arm_all();
    Serial.printf("[Motors] arm_all: %d\n", r);
    r = ESCCMD_start_timer();
    Serial.printf("[Motors] start_timer: %d\n", r);
    motor1.init(0);
    motor2.init(1);
}

void MotorManager::on(float throttle) {
    motor1.on(throttle);
    motor2.on(throttle);
}

void MotorManager::on(float throttle1, float throttle2) {
    motor1.on(throttle1);
    motor2.on(throttle2);
}

void MotorManager::off(int motor) {
    if (motor == 1)      motor1.off();
    else if (motor == 2) motor2.off();
    else                 { motor1.off(); motor2.off(); }
}

bool MotorManager::getRPM(int motor, int16_t &rpm) {
    return ESCCMD_read_rpm(motor - 1, &rpm) == 0;
}

bool MotorManager::getVoltage(int motor, float &volts) {
    uint16_t raw;
    if (ESCCMD_read_volt(motor - 1, &raw) != 0) return false;
    volts = raw * 0.01f;
    return true;
}

bool MotorManager::getCurrent(int motor, float &amps) {
    uint16_t raw;
    if (ESCCMD_read_amp(motor - 1, &raw) != 0) return false;
    amps = raw * 0.01f;
    return true;
}

bool MotorManager::getTemp(int motor, uint8_t &degC) {
    return ESCCMD_read_deg(motor - 1, &degC) == 0;
}
