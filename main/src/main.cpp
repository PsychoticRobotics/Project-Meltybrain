//
// Created by Atharv Goel on 9/21/25.
// Updated last by Atharv Goel on 9/28/25.
//

#include "Accelerometer.h"
#include "../lib/Eigen/Dense"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>


AccelerometerManager accelerometers;

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

    init_motors();
}

void loop() {
    Serial.println("Fetching data...");

    // Fetch and output accelerometer data
    Vector3d accelData = accelerometers.fetch();
    Serial.print("Average Acceleration: ");
    Serial.print("X: ");
    Serial.print(accelData.x());
    Serial.print("g, Y: ");
    Serial.print(accelData.y());
    Serial.print("g, Z: ");
    Serial.print(accelData.z());
    Serial.println("g");

    // Add a delay to prevent overwhelming the serial output
    delay(100);  // Adjust the delay as needed (in milliseconds)

    float foo = 1;
    motor_1_on(foo);
    motor_2_on(foo);
    
    motor_1_coast();
    motor_2_coast();
}
