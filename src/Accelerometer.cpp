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

// Read sensors once and cache the result. Call at the start of each loop iteration.
// If a calibration manager is attached, the separation axis (y) of each
// individual cache is corrected before computing the average.
void AccelerometerManager::refresh() {
    if (!accel1.initialized) return;

    _cache1 = accel1.fetch();
    _cache2 = accel2.initialized ? accel2.fetch() : _cache1;

    // Apply runtime calibration to the y-axis (separation axis) of each sensor.
    // Corrects zero-G DC bias and range-dependent gain nonlinearity.
    // The x and z axes retain only the static setAdjustment() corrections.
    if (_cal) {
        _cache1.y() = _cal->apply(0, (float)_cache1.y());
        _cache2.y() = _cal->apply(1, (float)_cache2.y());
    }

    _cache = accel2.initialized ? (_cache1 + _cache2) * 0.5 : _cache1;
}

// Returns the averaged (or single-sensor) raw XYZ reading — call refresh() first.
Vector3d AccelerometerManager::fetchXYZ() {
    return _cache;
}

// Returns the averaged reading rotated into the Normal-Tangential-Up frame.
// N = centripetal (toward spin centre), T = tangential, U = up (out of arena plane).
// The 45° rotation in the x-z plane compensates for the sensor's physical mounting angle.
Vector3d AccelerometerManager::fetchNTU() {
    return {
        SQRT_2_OVER_2 * (_cache.x() + _cache.z()),  // N — centripetal
        _cache.y(),                                   // T — tangential
        SQRT_2_OVER_2 * (_cache.z() - _cache.x())   // U — vertical (out of arena plane)
    };
}

// Individual per-sensor raw readings — call refresh() first.
// Used by AngleEstimator for the differential centre-of-rotation calculation.
// If only one accelerometer is fitted, both return the same value.
Vector3d AccelerometerManager::fetchXYZ1() { return _cache1; }
Vector3d AccelerometerManager::fetchXYZ2() { return _cache2; }

// Blocking zero-G capture (~400 ms).
// Reads ACCEL_CAL_ZERO_SAMPLES raw samples directly from each sensor (bypassing
// the cached / calibrated path so the capture is self-consistent), averages them,
// and stores the result via the attached calibration manager.
// The robot MUST be stationary during this call.
void AccelerometerManager::captureZeroG() {
    if (!accel1.initialized || !_cal) return;

    Serial.println("[AccelCal] Capturing zero-G — keep robot still for ~400 ms...");

    float sum0 = 0.0f, sum1 = 0.0f;
    bool dual = accel2.initialized;

    for (uint16_t i = 0; i < ACCEL_CAL_ZERO_SAMPLES; i++) {
        // Read directly from sensor objects — NOT through the calibrated cache —
        // so the zero-G offset is measured in the same raw space that apply() uses.
        sum0 += (float)accel1.fetch().y();
        sum1 += dual ? (float)accel2.fetch().y() : (float)accel1.fetch().y();
        delay(2);  // ~2 ms per sample → 200 samples ≈ 400 ms total
    }

    float n = (float)ACCEL_CAL_ZERO_SAMPLES;
    _cal->captureZero(0, sum0 / n);
    _cal->captureZero(1, dual ? sum1 / n : sum0 / n);  // mirror if single-sensor

    Serial.println("[AccelCal] Zero-G capture complete.");
}

void AccelerometerManager::log(File& logger, uint32_t time) {
    if (!logger) return;

    logger.print(time);       logger.print(",");
    logger.print(_cache1.x()); logger.print(",");
    logger.print(_cache1.y()); logger.print(",");
    logger.print(_cache1.z()); logger.print(",");
    logger.print(_cache2.x()); logger.print(",");
    logger.print(_cache2.y()); logger.print(",");
    logger.println(_cache2.z());
}
