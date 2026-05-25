#include "Motor.h"
#include "ESCCMD.h"

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

void Motor::coast() {
    ESCCMD_throttle(motorIndex, 0);
}

// --- MotorManager ---

void MotorManager::init() {
    ESCCMD_init(2);
    ESCCMD_3D_on_silent(); // 3D mode must be pre-configured in ESC firmware (AM32 configurator)
    ESCCMD_arm_all();
    ESCCMD_start_timer();
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

void MotorManager::coast(int motor) {
    if (motor == 1)      motor1.coast();
    else if (motor == 2) motor2.coast();
    else                 { motor1.coast(); motor2.coast(); }
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
