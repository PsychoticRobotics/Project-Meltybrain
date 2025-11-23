#include "Motor.h"
#include <Arduino.h>

//TODO: implement some kind of idle state and spin state (so it doesn't fuck up)
//very far in the future TODO: backup in case spin fails
//TODO: implement turning math
//we will be getting data from both ir sensing and acceleration

const int PWM_1000us = 65536 * 40 / 100;
const int PWM_2000us = 65536 * 80 / 100;

int throttleToPWM(float input, bool stretch) {
    // Constrain the throttle value between 0 and 1
    input = constrain(input, -1.0, 1.0);
    /*// do non-linear stretch or not?
    if (stretch) {
        input = stretchThrottle(input);
    }*/
    // Map the throttle value (-1 to 1) to the PWM duty cycle values (1000us and 2000us converted to the 400Hz PWM)
    return map(input, -1, 1, PWM_1000us, PWM_2000us); //linear mapping 
}

// PRIVATE: set motor power during the powered phase
void Motor::motor_on(float motor_percent, int motor_pin) {
  // Simple fixed PWM
  analogWrite(motor_pin, throttleToPWM(motor_percent, false));

  // Optional: scale PWM by motor_percent if using dynamic throttle
  /* float throttle_pwm = PWM_MOTOR_COAST + motor_percent * (PWM_MOTOR_ON - PWM_MOTOR_COAST);
     if (throttle_pwm > PWM_MOTOR_ON) throttle_pwm = PWM_MOTOR_ON;
     analogWrite(motor_pin, throttle_pwm);
  */
}

// PUBLIC: turn motor 1 on
void Motor::motor_1_on(float motor_percent) {
  motor_on(motor_percent, MOTOR_PIN1);
}

// PUBLIC: turn motor 2 on
void Motor::motor_2_on(float motor_percent) {
  motor_on(motor_percent, MOTOR_PIN2);
}

// PUBLIC: turn motor 1 on
void Motor::motors_on(float motor_percent) {
  motor_on(motor_percent, MOTOR_PIN1);
  motor_on(motor_percent, MOTOR_PIN2);
}

// PRIVATE: coast phase
void Motor::motor_coast(int motor_pin) {
  analogWrite(motor_pin, PWM_MOTOR_COAST);
}


// PUBLIC: coast motor 1
void Motor::motor_1_coast() {
  motor_coast(MOTOR_PIN1);
}

// PUBLIC: coast motor 2
void Motor::motor_2_coast() {
  motor_coast(MOTOR_PIN2);
}

// PRIVATE: turn motor off
void Motor::motor_break(int motor_pin) {
  analogWrite(motor_pin, PWM_MOTOR_BREAK);
}

// PUBLIC: motor 1 off
void Motor::motor_1_break() {
  motor_break(MOTOR_PIN1);
}

// PUBLIC: motor 2 off
void Motor::motor_2_break() {
  motor_break(MOTOR_PIN2);
}

// PUBLIC: both motors off
void Motor::motors_break() {
  motor_1_break();
  motor_2_break();
}

// PUBLIC: initialize pins
void Motor::init_motors() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  motors_break();
}



