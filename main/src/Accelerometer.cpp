//
// Created by Atharv Goel on 9/20/25.
//

#include "../include/Accelerometer.h"
#include <Wire.h>

bool Accelerometer::init(uint8_t address) {
    /** Unsure if useful / functional */
    Wire.begin();
    // Wire.setClock(400000); // Increase I2C speed to read accelerometer data in ~1ms (supposedly)

    base.setI2CAddr(address); // Set the correct address so we're reading from the right thing
    base.begin(LIS331::USE_I2C); // We're using I2C

    /** This is not correct. Either remove or find the correct values for bad status. */
    uint8_t status = base.readReg(STATUS_REG); // Check the status to see if things look good
    if (status == 0xFF || status == 0x00) { // I'm guessing these are suspicious values but idrk since there's no documentation
        // return false;
    }

    base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
    base.setODR(LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)
    base.axesEnable(true); /** Test to see if necessary */

    initialized = true;
    return true;
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
        Serial.println("Accelerometer one initialized"); /** Add the actual address (e.g. 0x18) too*/
        success = true;
    }

    if (accel2.init(add2)) {
        Serial.println("Accelerometer two initialized"); /** Add the actual address (e.g. 0x18) too*/
        success = true;
    }

    return success; // Return true if at least one accelerometer initialized successfully
}

bool AccelerometerManager::setup(uint8_t add) {
    if (accel1.init(add)) {
        Serial.println("Accelerometer initialized"); /** Add the actual address (e.g. 0x18) too*/
        return true;
    }
    return false;
}

Vector3d AccelerometerManager::fetch() { // Return the average of the two accelerometers if both are initialized, otherwise return data from the one that is
    if (accel1.initialized) {
        if (!accel2.initialized)
            return accel1.fetch();
        return (accel1.fetch() + accel2.fetch()) / 2.0;
    } else
        return accel1.fetch();
}