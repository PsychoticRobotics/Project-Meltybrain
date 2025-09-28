//
// Created by Atharv Goel on 9/20/25.
// Updated last by Atharv Goel on 9/28/25.
//

#include "Accelerometer.h"
#include "Config.h"
#include <Wire.h>
#include <SPI.h>

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

    return Vector3d{ // Convert to g's and return
            base.convertToG(400, x), // 400 g's is the max range in HIGH_RANGE mode
            base.convertToG(400, y),
            base.convertToG(400, z)
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