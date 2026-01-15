#include "Accelerometer.h"
#include "Config.h"
#include <Wire.h>
#include <SPI.h>

const double GRAVITY = 9.81;
const double SQRT_2_OVER_2 = 0.70710678118;

bool Accelerometer::init(int addr) {
    switch (PROTOCOL) {
        case 0: // SPI
            pinMode(addr, OUTPUT); // Set the chip select addr
            digitalWrite(addr, HIGH);
            SPI.begin();

            base.setSPICSPin(addr); // Set the correct CS addr so we're reading from the right thing
            base.begin(LIS331::USE_SPI); // We're using SPI

            base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
            base.setODR(LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)
            break;
        case 1: // I2C
            base.setI2CAddr(addr); // Set the correct add so we're reading from the right thing
            base.begin(LIS331::USE_I2C); // We're using I2C

            base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
            base.setODR(LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)
            return false;
    }
    initialized = true;
    return true; // Ideally return false if initialization fails, however, there is no way of checking :(
}

Vector3d Accelerometer::fetch() {
    int16_t x, y, z;
    base.readAxes(x, y, z); // Read the raw values
    Serial.println("RAW x: " + String(x) + " y: " + String(y) + " z: " + String(z));

    return Vector3d{ // Convert to g's and return
            base.convertToG(400, x) * GRAVITY, // 400 g's is the max range in HIGH_RANGE mode
            base.convertToG(400, y) * GRAVITY,
            base.convertToG(400, z) * GRAVITY
    };
}

bool AccelerometerManager::init(int addr1, int addr2) {
    bool success = false;
    switch (PROTOCOL) {
        case 0: // SPI
            if (accel1.init(addr1)) {
                Serial.println("Accelerometer one initialized at pin " + String(addr1));
                success = true;
            }
            if (accel2.init(addr2)) {
                Serial.println("Accelerometer two initialized at pin " + String(addr2));
                success = true;
            }
            break;
        case 1: // I2C
            if (accel1.init((uint8_t) addr1)) {
                Serial.println("Accelerometer one initialized at 0x" + String(addr1, HEX));
                success = true;
            }
            if (accel2.init((uint8_t) addr2)) {
                Serial.println("Accelerometer two initialized at 0x" + String(addr2, HEX));
                success = true;
            }
            break;
    }
    if (!success)
        Serial.println("Failed to initialize any accelerometers");
    return success; // Return true if at least one accelerometer initialized successfully
}

bool AccelerometerManager::init(int addr) {
    switch (PROTOCOL) {
        case 0: // SPI
            if (accel1.init(addr)) {
                Serial.println("Accelerometer initialized at addr " + String(addr));
                return true;
            }
            break;
        case 1: // I2C
            if (accel1.init((uint8_t) addr)) {
                Serial.println("Accelerometer initialized at 0x" + String(addr, HEX));
                return true;
            }
            break;
    }
    Serial.println("Failed to initialize accelerometer");
    return false;
}

// Return the average of the two accelerometers if both are initialized,
// otherwise return data from the one that is
// Raw X, Y, Z of the accelerometers
Vector3d AccelerometerManager::fetchXYZ() {
    assert(accel1.initialized);
    Vector3d data;
    if (accel2.initialized)
        data = (accel1.fetch() + accel2.fetch()) / 2.0;
    else
        data = accel1.fetch();

    return data;
}

// Return the average of the two accelerometers if both are initialized,
// otherwise return data from the one that is
// Raw normal, tangential, up of the accelerometers
Vector3d AccelerometerManager::fetchNTU() {
    assert(accel1.initialized);
    Vector3d data;
    if (accel2.initialized)
        data = (accel1.fetch() + accel2.fetch()) / 2.0;
    else
        data = accel1.fetch();

    return {SQRT_2_OVER_2 * (data.y() + data.z()), data.x(), SQRT_2_OVER_2 * (data.y() - data.z())};
}