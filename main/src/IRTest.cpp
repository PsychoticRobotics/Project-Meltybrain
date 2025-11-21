#include <Arduino.h>

const int IR_LED_PIN = 9;
const int IR_RECEIVER_PIN = 2;

// Pulse timing
const int PULSE_WIDTH_US = 10;     // 10 µs laser pulse
const unsigned long MAX_WAIT = 1000; // max wait for return in µs

enum IRState { IDLE, PULSED, WAIT_FOR_RETURN };
IRState irState = IDLE;

elapsedMicros timer;
unsigned long tof_us = 0;

void setup() {
    Serial.begin(115200);
    pinMode(IR_LED_PIN, OUTPUT);
    pinMode(IR_RECEIVER_PIN, INPUT);
}

void loop() {
    switch(irState) {
        case IDLE:
            // Pulse the laser
            digitalWrite(IR_LED_PIN, HIGH);
            timer = 0;
            irState = PULSED;
            break;

        case PULSED:
            if (timer >= PULSE_WIDTH_US) {
                digitalWrite(IR_LED_PIN, LOW);
                timer = 0;      // reset timer for TOF
                irState = WAIT_FOR_RETURN;
            }
            break;

        case WAIT_FOR_RETURN:
            if (digitalRead(IR_RECEIVER_PIN) == HIGH) {
                tof_us = timer;   // record time-of-flight
                float distance_m = tof_us * 1e-6 * 3e8 / 2.0;
                Serial.printf("TOF: %lu us, Distance: %.3f m\n", tof_us, distance_m);
                irState = IDLE;
                delay(100); // optional delay between pulses
            } else if (timer > MAX_WAIT) {
                Serial.println("Timeout: no return detected");
                irState = IDLE;
            }
            break;
    }
}