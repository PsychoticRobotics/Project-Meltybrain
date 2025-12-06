//
// Created by Atharv Goel on 9/21/25.
// Updated last by Peter Elmer on 10/20/25.
//

#include "Accelerometer.h"
#include "Motor.h"
#include "Robot.h"
#include "../lib/Eigen/Dense"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>


AccelerometerManager accelerometers;
MotorManager motors;
Robot robot(accelerometers, motors);

unsigned previousTime = 0;
unsigned currentTime = 0;

[[noreturn]] void quit() {
    Serial.println("Exiting program.");
    while (true) {
        // Infinite loop to halt execution
    }
}

void setup() {
    Serial.begin(19200);
    switch (PROTOCOL) {
        case 0: // SPI
            pinMode(MOSI, OUTPUT);
            pinMode(MISO, INPUT);
            pinMode(SCK, OUTPUT);

            if (!accelerometers.setup(10)) quit(); // Initialize accelerometer with CS pin 10

            break;
        case 1: // I2C
            Wire.begin();
            Wire.setClock(400000); // I2C fast mode

            if (!accelerometers.setup(0x19, 0x18)) quit(); // Initialize accelerometer with I2C address 0x19

            break;
        default:
            Serial.println("Invalid protocol specified in Config.h");
            quit();
    }

    motors.init(14, 15);
    delay(1000); // Motors/ESCs need at least 1 second to arm
}

void loop() {
    currentTime = micros();
    robot.move(0.0f, 0.0f, 0.0f, currentTime - previousTime);
    previousTime = currentTime;
}