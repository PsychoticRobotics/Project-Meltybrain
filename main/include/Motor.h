#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

// Motor pins
#ifndef MOTOR_PIN1
#define MOTOR_PIN1 2
#endif

#ifndef MOTOR_PIN2
#define MOTOR_PIN2 3
#endif



// PWM values
#define PWM_MOTOR_ON 230 //hi
#define PWM_MOTOR_COAST 100
#define PWM_MOTOR_BREAK 100 //lo

class Motor {
public:
    Motor() = default;

    // Initialize motors
    void init_motors();

    // Public motor control functions
    void motor_1_on(float throttle_percent);
    void motor_2_on(float throttle_percent);

    void motor_1_break();
    void motor_2_break();
    void motors_break();

    void motor_1_coast();
    void motor_2_coast();




private:
    // Private helper functions
    void motor_on(float throttle_percent, int motor_pin);
    void motor_break(int motor_pin);
    void motor_coast(int motor_pin);
};

#endif // MOTOR_H