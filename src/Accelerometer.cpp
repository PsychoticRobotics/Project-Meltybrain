#include "Accelerometer.h"
#include "Config.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

const double GRAVITY = 9.81;
const double SQRT_2_OVER_2 = 0.70710678118;

void Accelerometer::init(int addr) {
    switch (PROTOCOL) {
        case 0: // SPI
            pinMode(addr, OUTPUT);
            digitalWrite(addr, HIGH);
            SPI.begin();

            base.setSPICSPin(addr);
            base.begin(LIS331::USE_SPI);

            base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
            base.setODR(LIS331::DR_1000HZ);
            break;
        case 1: // I2C
            base.setI2CAddr(addr);
            base.begin(LIS331::USE_I2C);

            base.setFullScale(LIS331::HIGH_RANGE); // +/- 400g range
            base.setODR(LIS331::DR_1000HZ); // Getting data as frequently as possible will hopefully minimize drift (but might amplify noise)
            break;
        default:
            Serial.println("Error: Accelerometer::init - Unknown PROTOCOL selected");
    }
    initialized = true;
}

// Set per-axis offset (additive) and scale (multiplicative) for this accelerometer.
// Applied as: adjusted = (raw + offset) * scale
void Accelerometer::setAdjustment(Vector3d offset, Vector3d scale) {
    _offset = offset;
    _scale  = scale;
}

Vector3d Accelerometer::fetch() {
    int16_t x, y, z;
    base.readAxes(x, y, z);

    Vector3d raw{
        base.convertToG(400, x),
        base.convertToG(400, y),
        base.convertToG(400, z)
    };

    // Apply per-accelerometer offset then scale
    Vector3d adjusted{
        (raw.x() + _offset.x()) * _scale.x(),
        (raw.y() + _offset.y()) * _scale.y(),
        (raw.z() + _offset.z()) * _scale.z()
    };

    return adjusted;
}

// ─── AccelerometerManager ────────────────────────────────────────────────────

void AccelerometerManager::init(int addr1, int addr2) {
    switch (PROTOCOL) {
        case 0: // SPI
            accel1.init(addr1);
            Serial.println("Accelerometer one initialized at pin " + String(addr1));
            accel2.init(addr2);
            Serial.println("Accelerometer two initialized at pin " + String(addr2));
            break;
        case 1: // I2C
            accel1.init((uint8_t) addr1);
            Serial.println("Accelerometer one initialized at 0x" + String(addr1, HEX));
            accel2.init((uint8_t) addr2);
            Serial.println("Accelerometer two initialized at 0x" + String(addr2, HEX));
            break;
        default:
             Serial.println("Error: AccelerometerManager::init (2 args) - Unknown PROTOCOL");
             break;
    }
}

void AccelerometerManager::init(int addr) {
    switch (PROTOCOL) {
        case 0: // SPI
            accel1.init(addr);
            Serial.println("Accelerometer initialized at addr " + String(addr));
            break;
        case 1: // I2C
            accel1.init((uint8_t) addr);
            Serial.println("Accelerometer initialized at 0x" + String(addr, HEX));
            break;
        default:
             Serial.println("Error: AccelerometerManager::init (1 arg) - Unknown PROTOCOL");
             break;
    }
}

// Set per-accelerometer adjustments independently.
// offset: additive correction per axis  (e.g. bias zeroing)
// scale:  multiplicative correction per axis (e.g. sensitivity mismatch)
// Applied as: adjusted = (raw + offset) * scale
void AccelerometerManager::setAdjustments(
    Vector3d offset1, Vector3d scale1,
    Vector3d offset2, Vector3d scale2)
{
    accel1.setAdjustment(offset1, scale1);
    accel2.setAdjustment(offset2, scale2);
}

// Return the average of both accelerometers if both initialized,
// otherwise return data from accel1.
// Raw X, Y, Z
Vector3d AccelerometerManager::fetchXYZ() {
    if (!accel1.initialized) {
        Serial.println("ERROR: fetchXYZ() called but accelerometer not initialized.");
        // Return a zero vector to prevent further issues
        return Vector3d{0, 0, 0};
    }
    Vector3d data;
    if (accel2.initialized)
        data = (accel1.fetch() + accel2.fetch()) / 2.0;
    else
        data = accel1.fetch();

    return data;
}

// Return the average of both accelerometers if both initialized,
// otherwise return data from accel1.
// Rotated into Normal, Tangential, Up frame
Vector3d AccelerometerManager::fetchNTU() {
    if (!accel1.initialized) {
        Serial.println("FATAL: fetchNTU() called but accelerometer not initialized. Halting.");
        // This is a fatal error for the robot's logic.
        while(1);
    }
    Vector3d data;
    if (accel2.initialized)
        data = (accel1.fetch() + accel2.fetch()) / 2.0;
    else
        data = accel1.fetch();

    Serial.print("Accelerometer: ");
    Serial.print("x: "); Serial.print(data.x());
    Serial.print(" y: "); Serial.print(data.y());
    Serial.print(" z: "); Serial.println(data.z());

    return {
        SQRT_2_OVER_2 * (data.x() + data.z()),
        data.y(),
        SQRT_2_OVER_2 * (data.z() - data.x())
    };
}

void AccelerometerManager::log(File& logger, uint32_t time) {
    if (!logger) return;

    Vector3d d1 = accel1.fetch();
    Vector3d d2;
    if (accel2.initialized) {
        d2 = accel2.fetch();
    } else {
        d2.setZero();
    }

    logger.print(time);   logger.print(",");
    logger.print(d1.x()); logger.print(",");
    logger.print(d1.y()); logger.print(",");
    logger.print(d1.z()); logger.print(",");
    logger.print(d2.x()); logger.print(",");
    logger.print(d2.y()); logger.print(",");
    logger.println(d2.z());
}
