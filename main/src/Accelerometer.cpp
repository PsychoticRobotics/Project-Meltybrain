//
// Created by Atharv Goel on 9/20/25.
// Updated last by Peter Elmer on 10/20/25.
//

/* This file only outputs unfiltered Accelerometer data */

#include "Accelerometer.h"
#include "Config.h"
#include <Wire.h>
#include <SPI.h>

const double GRAVITY = 9.81;
const double SQRT_2_OVER_2 = 0.70710678118;

bool Accelerometer::init(uint8_t add) {
    base.setI2CAddr(add); // Set the correct add so we're reading from the right thing
    base.begin(LIS331::USE_I2C); // We're using I2C

    base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
    base.setODR(LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)

    initialized = true;
    return true; // Ideally return false if initialization fails, however, there is no way of checking :(
}

bool Accelerometer::init(int CS) {
    pinMode(CS, OUTPUT); // Set the chip select pin
    digitalWrite(CS, HIGH);
    SPI.begin();

    base.setSPICSPin(CS); // Set the correct CS pin so we're reading from the right thing
    base.begin(LIS331::USE_SPI); // We're using SPI

    base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
    base.setODR(LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)

    initialized = true;
    return true; // Ideally return false if initialization fails, however, there is no way of checking :(
}

Vector3d Accelerometer::fetch() {
    int16_t x, y, z;
    base.readAxes(x, y, z); // Read the raw values

    return Vector3d{ // Convert to g's and return (original), made it return straight m/s^2 (* GRAVITY)
            base.convertToG(400, x) * GRAVITY, // 400 g's is the max range in HIGH_RANGE mode
            base.convertToG(400, y) * GRAVITY,
            base.convertToG(400, z) * GRAVITY
    };
}

bool AccelerometerManager::setup(int in1, int in2) {
    bool success = false;
    switch (PROTOCOL) {
        case 0: // SPI
            if (accel1.init(in1)) {
                Serial.println("Accelerometer one initialized at pin " + String(in1));
                success = true;
            }

            if (accel2.init(in2)) {
                Serial.println("Accelerometer two initialized at pin " + String(in2));
                success = true;
            }
            break;
        case 1: // I2C
            if (accel1.init(in1)) {
                Serial.println("Accelerometer one initialized at 0x" + String(in1, HEX));
                success = true;
            }

            if (accel2.init(in2)) {
                Serial.println("Accelerometer two initialized at 0x" + String(in2, HEX));
                success = true;
            }
            break;
    }
    if (!success)
        Serial.println("Failed to initialize any accelerometers");
    return success; // Return true if at least one accelerometer initialized successfully
}

bool AccelerometerManager::setup(int in) {
    switch (PROTOCOL) {
        case 0: // SPI
            if (accel1.init(in)) {
                Serial.println("Accelerometer initialized at pin " + String(in));
                return true;
            }
            break;
        case 1: // I2C
            if (accel1.init(in)) {
                Serial.println("Accelerometer initialized at 0x" + String(in, HEX));
                return true;
            }
            break;
    }
    Serial.println("Failed to initialize accelerometer");
    return false;
}

// Return the average of the two accelerometers if both are initialized,
// otherwise return data from the one that is
Vector3d AccelerometerManager::fetch() {
    if (accel1.initialized) {
        if (!accel2.initialized)
            return accel1.fetch();
        return (accel1.fetch() + accel2.fetch()) / 2.0;
    } else
        return accel1.fetch();
}

// Both fetches and prints (x, y, z)
Vector3d AccelerometerManager::print1() {
    // --- Determine how many accelerometers are initialized ---
    int initializedCount = (accel1.initialized ? 1 : 0) + (accel2.initialized ? 1 : 0);

    // --- Compute the acceleration vector ---
    Vector3d accelData;
    switch (initializedCount) {
        case 0:
            accelData = Vector3d{0.0, 0.0, 0.0};  // No sensors initialized
            break;

        case 1:
            accelData = accel1.initialized ? accel1.fetch() : accel2.fetch();
            break;

        case 2:
            accelData = (accel1.fetch() + accel2.fetch()) / 2.0;
            break;

        default:
            accelData = Vector3d{0.0, 0.0, 0.0};  // Should never happen
            break;
    }

    // --- Print timestamped accelerometer values ---
    unsigned long t = micros();   // timestamp in microseconds
    Serial.printf("[%010lu us]  Accel (m/s^2)  X: %.6f, Y: %.6f, Z: %.6f\n",
                  t,
                  accelData.x(),
                  accelData.y(),
                  accelData.z());

    // --- Return the vector ---
    return accelData;
}

// Both fetches and prints (normal, tangential, up)
Vector3d AccelerometerManager::print2() {
    // --- Determine how many accelerometers are initialized ---
    int initializedCount = (accel1.initialized ? 1 : 0) + (accel2.initialized ? 1 : 0);

    // --- Compute the acceleration vector ---
    Vector3d accelData;
    switch (initializedCount) {
        case 0:
            accelData = Vector3d{0.0, 0.0, 0.0};  // No sensors initialized
            break;

        case 1:
            accelData = accel1.initialized ? accel1.fetch() : accel2.fetch();
            accelData = Vector3d(
                        SQRT_2_OVER_2 * (accelData.z() + accelData.y()), // (√2/2 * z) + (√2/2 * y)
                        accelData.x(),
                        SQRT_2_OVER_2 * (accelData.z() + accelData.y()) // (√2/2 * z) - (√2/2 * y)
                        );
            break;

        case 2:
            accelData = (accel1.fetch() + accel2.fetch()) / 2.0;
            accelData = Vector3d(
                        SQRT_2_OVER_2 * (accelData.z() + accelData.y()), // (√2/2 * z) + (√2/2 * y)
                        accelData.x(),
                        SQRT_2_OVER_2 * (accelData.z() + accelData.y()) // (√2/2 * z) - (√2/2 * y)
                        ); 
            break;

        default:
            accelData = Vector3d{0.0, 0.0, 0.0};  // Should never happen
            break;
    }

    // --- Print timestamped accelerometer values ---
    unsigned long t = micros();   // timestamp in microseconds
    Serial.printf("[%010lu us]  Accel (m/s^2)  Normal: %.6f, Tangential: %.6f, Up: %.6f\n",
                  t,
                  accelData.x(),
                  accelData.y(),
                  accelData.z());

    // --- Return the vector ---
    return accelData;
}