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


AccelerometerManager accelerometers;
Motor motors;

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

            if (!accelerometers.setup(0x19)) quit(); // Initialize accelerometer with I2C address 0x19

            break;
        default:
            Serial.println("Invalid protocol specified in Config.h");
            quit();
    }

    motors.init_motors();
}

void loop() {
    Serial.println("Fetching data...");

    // Fetch and output accelerometer data
    accelerometers.print(); // fetches and prints
    delay(100);

    float low = 0;
    float high = 1;

    motors.motor_1_on(foo);
    motors.motor_2_on(foo);
    
    motors.motor_1_coast();
    motors.motor_2_coast();
}

// Trying stuff:
/*
Vector3d accelData;
Vector3d gyroData;
Vector3d magData;

void readAccel() { accelData = accelerometers.fetch(); }
void readGyro()  { gyroData  = gyros.fetch(); }
void readMag()   { magData   = magnetometer.fetch(); }
*/

// Old code:
/*

    Vector3d accelData = accelerometers.fetch();
    Serial.print("Average Acceleration: ");
    // Serial.print("X: ");
    // Serial.print(accelData.x());
    Serial.printf("X: %dg", accelData.x()); // Delete this if it doesn't work
    // Serial.print("g, Y: ");
    // Serial.print(accelData.y());
    Serial.printf("Y: %dg", accelData.y()); // Delete this if it doesn't work
    // Serial.print("g, Z: ");
    // Serial.print(accelData.z());
    Serial.printf("Z: %dg", accelData.z()); // Delete this if it doesn't work
    // Serial.println("g");
    
*/