#include "Accelerometer.h"
#include "Config.h"
#include <Wire.h>

const double GRAVITY = 9.81;

void Accelerometer::init(int addr) {
    base.setI2CAddr(addr);
    base.begin(LIS331::USE_I2C);
    base.setFullScale(LIS331::HIGH_RANGE);   // ±400 g range
    base.setODR(LIS331::DR_1000HZ);
    initialized = true;
}

// Set per-axis offset (additive) and scale (multiplicative) for this accelerometer.
// Applied as: adjusted = (raw + offset) * scale
void Accelerometer::setAdjustment(Vec3d offset, Vec3d scale) {
    _offset = offset;
    _scale  = scale;
}

Vec3d Accelerometer::fetch() {
    // Physical axis convention (both sensors mounted identically):
    //   x → right  (positive = toward the right side of the robot)
    //   y → forward (positive = toward the front of the robot)
    //   z → up     (positive = away from the arena floor)
    int16_t x, y, z;
    base.readAxes(x, y, z);

    Vec3d raw{
        base.convertToG(400, x),  // right
        base.convertToG(400, y),  // forward
        base.convertToG(400, z)   // up
    };

    // Apply per-accelerometer offset then scale
    Vec3d adjusted{
        (raw.x() + _offset.x()) * _scale.x(),
        (raw.y() + _offset.y()) * _scale.y(),
        (raw.z() + _offset.z()) * _scale.z()
    };

    return adjusted;
}

// ─── AccelerometerManager ────────────────────────────────────────────────────

void AccelerometerManager::init(int addr1, int addr2) {
    accel1.init(addr1);
    Serial.println("Accelerometer 1 initialized at 0x" + String(addr1, HEX));
    accel2.init(addr2);
    Serial.println("Accelerometer 2 initialized at 0x" + String(addr2, HEX));
}

void AccelerometerManager::init(int addr) {
    accel1.init(addr);
    Serial.println("Accelerometer initialized at 0x" + String(addr, HEX));
}

// Set per-accelerometer adjustments independently.
// offset: additive correction per axis  (e.g. bias zeroing)
// scale:  multiplicative correction per axis (e.g. sensitivity mismatch)
// Applied as: adjusted = (raw + offset) * scale
void AccelerometerManager::setAdjustments(
    Vec3d offset1, Vec3d scale1,
    Vec3d offset2, Vec3d scale2)
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
Vec3d AccelerometerManager::fetchXYZ() {
    return _cache;
}

// Individual per-sensor raw readings — call refresh() first.
// Used by AngleEstimator for the differential centre-of-rotation calculation.
// If only one accelerometer is fitted, both return the same value.
Vec3d AccelerometerManager::fetchXYZ1() { return _cache1; }
Vec3d AccelerometerManager::fetchXYZ2() { return _cache2; }

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

