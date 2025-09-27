//
// Created by Atharv Goel on 9/21/25.
//

#include "../include/Accelerometer.h"
#include "../lib/Eigen/Dense"
#include <Arduino.h>


AccelerometerManager accelerometers;

[[noreturn]] void quit() {
    Serial.println("Exiting program.");
    while (true) {
        // Infinite loop to halt execution
    }
}

void setup() {
    Serial.begin(19200);
    if (!accelerometers.setup(0x19)) quit(); // Initialize accelerometer
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
}