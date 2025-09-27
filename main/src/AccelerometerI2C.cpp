//
// Created by Atharv Goel on 9/20/25.
// Updated last by Atharv Goel on 9/27/25.
//

#include "../include/AccelerometerI2C.h"
#include <Wire.h>

bool Accelerometer::init(uint8_t address) {
    base.setI2CAddr(address); // Set the correct address so we're reading from the right thing
    base.begin(LIS331::USE_I2C); // We're using I2C

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

bool AccelerometerManager::setup(uint8_t add1, uint8_t add2) {
    bool success = false;

    if (accel1.init(add1)) {
        Serial.println("Accelerometer one initialized at 0x" + String(add1, HEX));
        success = true;
    }

    if (accel2.init(add2)) {
        Serial.println("Accelerometer two initialized at 0x" + String(add2, HEX));
        success = true;
    }

    return success; // Return true if at least one accelerometer initialized successfully
}

bool AccelerometerManager::setup(uint8_t add) {
    if (accel1.init(add)) {
        Serial.println("Accelerometer initialized at 0x" + String(add, HEX));
        return true;
    }
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