//
// Created by Atharv Goel on 9/21/25.
// Updated last by Peter Elmer on 10/20/25.
//

#include "Accelerometer.h"
#include "Motor.h"
#include "SensorFusion.h"
#include "../lib/Eigen/Dense"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>


// AccelerometerManager accelerometers;
MotorManager motors;

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

//            if (!accelerometers.setup(10)) quit(); // Initialize accelerometer with CS pin 10

            break;
        case 1: // I2C
            Wire.begin();
            Wire.setClock(400000); // I2C fast mode

//            if (!accelerometers.setup(0x19, 0x18)) quit(); // Initialize accelerometer with I2C address 0x19

            break;
        default:
            Serial.println("Invalid protocol specified in Config.h");
            quit();
    }

    motors.init(14, 15);
    delay(1000); // Motors/ESCs need at least 1 second to arm
    motors.on(15, 15);
    delay(5000);
    motors.off();
}

void loop() {
//    Serial.println("Fetching data...");

//    // Fetch and output accelerometer data
//    Vector3d accelData = accelerometers.fetch();
//    Serial.print("Average Acceleration: ");
//    Serial.print("X: ");
//    Serial.print(accelData.x());
//    // Serial.printf("X: %dg", accelData.x()); // Delete this if it doesn't work
//    Serial.print("g, Y: ");
//    Serial.print(accelData.y());
//    // Serial.printf("Y: %dg", accelData.y()); // Delete this if it doesn't work
//    Serial.print("g, Z: ");
//    Serial.print(accelData.z());
//    // Serial.printf("Z: %dg", accelData.z()); // Delete this if it doesn't work
//    Serial.println("g");
//
//    delay(100);
}