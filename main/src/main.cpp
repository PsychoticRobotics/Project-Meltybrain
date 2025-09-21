//
// Created by Atharv Goel on 9/21/25.
//

#include "../include/Accelerometer.h"
#include "../lib/Eigen/Dense"
#include <Arduino.h>   // Required for Arduino-style code (includes Serial)


Accelerometer accelSix;
Accelerometer accelSeven;

void setup() {
   Serial.begin(19200);

   // Initialize accelerometers
   if (!accelSix.init(0x18)) {
      Serial.println("Accelerometer six dead o6");
      while (true);  // Halt the program here
   }

   if (!accelSeven.init(0x18)) {
      Serial.println("Accelerometer seven dead o7");
      while (true);  // Halt the program here
   }
}

void loop() {
   Serial.println("Fetching data...");

   // Fetch accelerometer data
   Vector3d accelSixData = accelSix.fetch();
   Vector3d accelSevenData = accelSeven.fetch();
   Vector3d averageData = (accelSixData + accelSevenData) / 2.0;

   // Output the average acceleration data to the Serial Monitor
   Serial.print("Average Acceleration: ");
   Serial.print("X: ");
   Serial.print(averageData.x());
   Serial.print("g, Y: ");
   Serial.print(averageData.y());
   Serial.print("g, Z: ");
   Serial.print(averageData.z());
   Serial.println("g");

   // Add a delay to prevent overwhelming the serial output
   delay(100);  // Adjust the delay as needed (in milliseconds)
}