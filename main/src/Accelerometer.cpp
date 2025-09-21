//
// Created by Atharv Goel on 9/20/25.
//

#include "../include/Accelerometer.h"
#include <Wire.h>

bool Accelerometer::init(uint8_t address) {
    Wire.begin();
    Wire.setClock(400000); // Increase I2C speed to read accelerometer data in ~1ms (supposedly)

    base.setI2CAddr(address); // Set the correct address so we're reading from the right thing
    base.begin(LIS331::USE_I2C); // We're using I2C

    uint8_t status = base.readReg(STATUS_REG); // Check the status to see if things look good
    if (status == 0xFF ||
        status == 0x00) { // I'm guessing these are suspicious values but idrk since there's no documentation
        return false;
    }

    base.setFullScale(LIS331::HIGH_RANGE); // We're definitely going to experience more than 100 or 200 g's
    base.setODR(
            LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)

    return true;
}

Vector3d Accelerometer::fetch() {
    int16_t x, y, z;
    base.readAxes(x, y, z); // Read the raw values

    return Vector3d{ // Convert to g's and return
            base.convertToG(400, x),
            base.convertToG(400, y),
            base.convertToG(400, z)
    };
}
