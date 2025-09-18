#include <Arduino.h>
#include <math.h>
#include "SimpleMelt.h"
#include "SimpleMeltUtility.h"

// ------- Meltybrain core update loop --------

// Called every control frame in melty mode.
// Handles odometry, angle estimation, joystick input, and sets motor power.
void SimpleMelt::meltyStateUpdate() {

    // Calculate joystick angle (not used for omni, just forward/back)
    float stick_angle = atan2(throttle, 0); // Only throttle is used here
    float stick_magnitude = fconstrain(magnitude(0, throttle), 0, 1);
    spin_power = fconstrain(spin_power, 0, 1);

    // Timing: measure time since last update
    uint64_t time_step;
    if (micros() < previous_melty_frame_us) time_step = 0;
    else time_step = micros() - previous_melty_frame_us;
    previous_melty_frame_us = micros();
    if (time_step > 500000) time_step = 0; // Ignore bad frames (mode switch)

    // Calculate total acceleration magnitude using accelerometer data
    float cen_accel = sqrt(accelerometer_x * accelerometer_x +
                           accelerometer_y * accelerometer_y +
                           accelerometer_z * accelerometer_z);

    // Estimate angular velocity from centripetal acceleration
    float angular_vel = (reversed ? -1 : 1) * sqrt(fabs(cen_accel / (accelerometer_radius + radius_trim)));

    // Integrate angle using trapezoidal rule (Riemann sum)
    float delta_angle = (angular_vel + previous_ang_vel) * 0.5 * time_step * 0.000001;
    angle = fmod(angle + delta_angle, M_TWOPI);
    previous_ang_vel = angular_vel;

    // Apply user-requested heading rotation (stick right = rotate)
    angle -= rotation * M_TWOPI * turn_speed * time_step * 0.000001;

    // Calculate shortest angle difference from robot to stick ([-PI, PI))
    float angle_diff = modulo((stick_angle - angle) + M_PI, M_TWOPI) - M_PI;

    // LED arc: only light up LEDs on a certain arc of the spin
    float melty_led_offset = (!reversed) ? melty_led_offset_CW : melty_led_offset_CCW;
    float melty_led_angle = modulo(angle - melty_led_offset, M_TWOPI);
    melty_led = M_PI_4 < melty_led_angle && melty_led_angle < M_3PI_4;

    // Motor power: modulate for stick input (deflection = drive offset)
    float deflection = spin_power * lerp(stick_magnitude, 0, 1, 0, 0.5);
    if (angle_diff < 0) {
        // One side gets more power, one less, based on drive direction
        motor_power_foo = -1 * (spin_power + deflection * 3) * (reversed ? 1 : -1);
        motor_power_bar =      (spin_power - deflection * 1) * (reversed ? 1 : -1);
    } else {
        motor_power_foo = -1 * (spin_power - deflection * 1) * (reversed ? 1 : -1);
        motor_power_bar =      (spin_power + deflection * 3) * (reversed ? 1 : -1);
    }

    // Status LED: solid green when connected
    status_led = true;
}

// -------- Magnetometer-based melty update --------

// Global variables for filtering magnetometer readings
float comp_xy = 0;
float comp_z = 0;
float comp_atan2 = 0;
float alpha_xy = 0.75;
float alpha_z = 0.75;
float alpha_arctan2 = 0.75;

// Alternative update loop using magnetometer for heading estimation
void SimpleMelt::meltyMagStateUpdate() {

    // Joystick input as before
    float stick_angle = atan2(throttle, 0);
    float stick_magnitude = fconstrain(magnitude(0, throttle), 0, 1);
    spin_power = fconstrain(spin_power, 0, 1);

    // Timing as before
    uint64_t time_step;
    if (micros() < previous_melty_frame_us) time_step = 0;
    else time_step = micros() - previous_melty_frame_us;
    previous_melty_frame_us = micros();
    if (time_step > 500000) time_step = 0;

    // -------- Magnetometer filtering and heading ----------

    // Combine XY magnetometer axes for heading
    float magnetometer_xy = 0.7071 * (magnetometer_x + magnetometer_y);

    // Simple exponential moving average ("leaky integrator") filtering
    comp_xy = (comp_xy * alpha_xy) + (magnetometer_xy * (1 - alpha_xy));
    float filtered_xy = magnetometer_xy - comp_xy;

    comp_z = (comp_z * alpha_z) + (magnetometer_z * (1 - alpha_z));
    float filtered_z = magnetometer_z - comp_z;

    // Heading estimate from filtered mag axes
    angle = atan2(filtered_xy, filtered_z);

    // -----------------------------------------------------

    // User-requested heading change
    angle -= rotation * M_TWOPI * turn_speed * time_step * 0.000001;

    // Stick angle difference and LED arc logic, same as above
    float angle_diff = modulo((stick_angle - angle) + M_PI, M_TWOPI) - M_PI;
    float melty_led_offset = (!reversed) ? melty_led_offset_CW : melty_led_offset_CCW;
    float melty_led_angle = modulo(angle - melty_led_offset, M_TWOPI);
    melty_led = M_PI_4 < melty_led_angle && melty_led_angle < M_3PI_4;

    // Motor power logic, same as above
    float deflection = spin_power * lerp(stick_magnitude, 0, 1, 0, 0.5);
    if (angle_diff < 0) {
        motor_power_foo = -1 * (spin_power + deflection * 3) * (reversed ? 1 : -1);
        motor_power_bar =      (spin_power - deflection * 1) * (reversed ? 1 : -1);
    } else {
        motor_power_foo = -1 * (spin_power - deflection * 1) * (reversed ? 1 : -1);
        motor_power_bar =      (spin_power + deflection * 3) * (reversed ? 1 : -1);
    }

    // Status LED: solid green
    status_led = true;
}

// -------- Arcade drive (non-melty) mode --------

// Standard arcade drive for testing or fallback (no spinning)
// Also inverts controls for convenience
void SimpleMelt::arcadeStateUpdate() {
    throttle *= -1;
    rotation *= -1;
    float leftPower = 0;
    float rightPower = 0;
    float maxInput = (throttle > 0 ? 1 : -1) * max(fabs(throttle), fabs(rotation));
    if (throttle > 0) {
        if (rotation > 0) {
            leftPower = maxInput;
            rightPower = throttle - rotation;
        }
        else {
            leftPower = throttle + rotation;
            rightPower = maxInput;
        }
    } else {
        if (rotation > 0) {
            leftPower = maxInput;
            rightPower = throttle + rotation;
        }
        else {
            leftPower = throttle - rotation;
            rightPower = maxInput;
        }
    }

    melty_led = true; // Always on in arcade mode

    // Set motor power (scaled down for safety)
    motor_power_foo = leftPower * 0.1;
    motor_power_bar = rightPower * 0.1;

    // Status LED: solid green
    status_led = true;
}

// -------- Stop all motors, blink LEDs --------

// Called when connected but stopped (e.g. safety, disabled)
void SimpleMelt::stopStateUpdate() {
    // Blink melty LED three times per second
    int t = millis() % 1000;
    if (t < 100 || (t > 200 && t < 300) || (t > 400 && t < 500)) melty_led = true;
    else melty_led = false;

    // Turn off motors
    motor_power_foo = 0;
    motor_power_bar = 0;

    // Status LED: solid green
    status_led = true;
}

// -------- Disconnected: blink LEDs, stop motors --------

// Called when radio is disconnected
void SimpleMelt::disconnectedStateUpdate() {
    // Blink melty LED two times per second
    int t = millis() % 1000;
    if (t < 100 || (t > 200 && t < 300)) melty_led = true;
    else melty_led = false;

    // Turn off motors
    motor_power_foo = 0;
    motor_power_bar = 0;

    // Blink status LED: on/off every half second
    status_led = (millis() % 1000 < 500) ? false : true;
}
