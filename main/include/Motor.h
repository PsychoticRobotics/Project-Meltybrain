#ifndef MOTOR_PIN1 2
#ifndef MOTOR_PIN2 3

#define MOTOR_PIN1 2
#define MOTOR_PIN2 3

//stole these from openmelt we can mess around w/ values later
#define PWM_MOTOR_ON 230                          
#define PWM_MOTOR_COAST 100                   
#define PWM_MOTOR_OFF 100                  


class Motor {
public:
    Motor() = default;

    //intitialize motors
    void init_motors();

    //turn motor_X_on 
    void motor_1_on(float throttle_percent); // motify throttle_percent 
    void motor_2_on(float throttle_percent);

    //motors shut-down (robot not translating)
    void motor_1_off();
    void motor_2_off();
    void motors_off();

    //motors coasting (unpowered part of rotation when translating)
    void motor_1_coast();
    void motor_2_coast();
}