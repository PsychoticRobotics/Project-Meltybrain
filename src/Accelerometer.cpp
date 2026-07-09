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

    // Snapshot before runtime calibration so printDebug() can show both.
    _raw1 = _cache1;
    _raw2 = _cache2;

    if (_cal) {
        // x (separation axis, left-right): full correction — zero offset + gain table.
        _cache1.x() = _cal->apply (0, (float)_cache1.x());
        _cache2.x() = _cal->apply (1, (float)_cache2.x());
        // y (forward axis) and z (up axis): zero offset only.
        _cache1.y() = _cal->applyY(0, (float)_cache1.y());
        _cache2.y() = _cal->applyY(1, (float)_cache2.y());
        _cache1.z() = _cal->applyZ(0, (float)_cache1.z());
        _cache2.z() = _cal->applyZ(1, (float)_cache2.z());
    }

    _cache = accel2.initialized ? (_cache1 + _cache2) * 0.5 : _cache1;

    _ema1 = _ema1 * 0.99 + _cache1 * 0.01;
    _ema2 = _ema2 * 0.99 + _cache2 * 0.01;
}

// Returns the averaged (or single-sensor) raw XYZ reading — call refresh() first.
Vec3d AccelerometerManager::fetchXYZ() {
    return _cache;
}

void AccelerometerManager::printPlotter() const {
    // Teleplot format: one >label:value per line.
    // Axes: x=right (separation), y=forward, z=up.  Suffixes: r=raw, c=cal, e=ema.
    if (accel2.initialized) {
        Serial.printf(">A1xr:%.2f\n", (float)_raw1.x());
        Serial.printf(">A1xc:%.2f\n", (float)_cache1.x());
        Serial.printf(">A1xe:%.2f\n", (float)_ema1.x());
        Serial.printf(">A1yr:%.2f\n", (float)_raw1.y());
        Serial.printf(">A1yc:%.2f\n", (float)_cache1.y());
        Serial.printf(">A1ye:%.2f\n", (float)_ema1.y());
        Serial.printf(">A1zr:%.2f\n", (float)_raw1.z());
        Serial.printf(">A1zc:%.2f\n", (float)_cache1.z());
        Serial.printf(">A1ze:%.2f\n", (float)_ema1.z());
        Serial.printf(">A2xr:%.2f\n", (float)_raw2.x());
        Serial.printf(">A2xc:%.2f\n", (float)_cache2.x());
        Serial.printf(">A2xe:%.2f\n", (float)_ema2.x());
        Serial.printf(">A2yr:%.2f\n", (float)_raw2.y());
        Serial.printf(">A2yc:%.2f\n", (float)_cache2.y());
        Serial.printf(">A2ye:%.2f\n", (float)_ema2.y());
        Serial.printf(">A2zr:%.2f\n", (float)_raw2.z());
        Serial.printf(">A2zc:%.2f\n", (float)_cache2.z());
        Serial.printf(">A2ze:%.2f\n", (float)_ema2.z());
    } else {
        Serial.printf(">Axr:%.2f\n", (float)_raw1.x());
        Serial.printf(">Axc:%.2f\n", (float)_cache.x());
        Serial.printf(">Axe:%.2f\n", (float)_ema1.x());
        Serial.printf(">Ayr:%.2f\n", (float)_raw1.y());
        Serial.printf(">Ayc:%.2f\n", (float)_cache.y());
        Serial.printf(">Aye:%.2f\n", (float)_ema1.y());
        Serial.printf(">Azr:%.2f\n", (float)_raw1.z());
        Serial.printf(">Azc:%.2f\n", (float)_cache.z());
        Serial.printf(">Aze:%.2f\n", (float)_ema1.z());
    }
}

void AccelerometerManager::printDebug() const {
    // Sensor axes: x = right (separation), y = forward, z = up
    // raw = before runtime calibration;  cal = after
    // x: full correction (zero + gain table);  y/z: zero offset only
    if (accel2.initialized) {
        Serial.printf("[Accel1] x(raw)=%+7.2fg x(cal)=%+7.2fg x(ema)=%+7.2fg  y(raw)=%+7.2fg y(cal)=%+7.2fg y(ema)=%+7.2fg  z(raw)=%+7.2fg z(cal)=%+7.2fg z(ema)=%+7.2fg\n",
            (float)_raw1.x(), (float)_cache1.x(), (float)_ema1.x(),
            (float)_raw1.y(), (float)_cache1.y(), (float)_ema1.y(),
            (float)_raw1.z(), (float)_cache1.z(), (float)_ema1.z());
        Serial.printf("[Accel2] x(raw)=%+7.2fg x(cal)=%+7.2fg x(ema)=%+7.2fg  y(raw)=%+7.2fg y(cal)=%+7.2fg y(ema)=%+7.2fg  z(raw)=%+7.2fg z(cal)=%+7.2fg z(ema)=%+7.2fg\n",
            (float)_raw2.x(), (float)_cache2.x(), (float)_ema2.x(),
            (float)_raw2.y(), (float)_cache2.y(), (float)_ema2.y(),
            (float)_raw2.z(), (float)_cache2.z(), (float)_ema2.z());
    } else {
        Serial.printf("[Accel]  x(raw)=%+7.2fg x(cal)=%+7.2fg x(ema)=%+7.2fg  y(raw)=%+7.2fg y(cal)=%+7.2fg y(ema)=%+7.2fg  z(raw)=%+7.2fg z(cal)=%+7.2fg z(ema)=%+7.2fg\n",
            (float)_raw1.x(), (float)_cache.x(), (float)_ema1.x(),
            (float)_raw1.y(), (float)_cache.y(), (float)_ema1.y(),
            (float)_raw1.z(), (float)_cache.z(), (float)_ema1.z());
    }
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

    // Accumulate raw readings for both axes on both sensors.
    // Reading directly from sensor objects (not cached path) so the offsets are
    // measured in the same raw space that apply() / applyY() operate on.
    float sumX0 = 0.0f, sumX1 = 0.0f;
    float sumY0 = 0.0f, sumY1 = 0.0f;
    float sumZ0 = 0.0f, sumZ1 = 0.0f;
    bool dual = accel2.initialized;

    for (uint16_t i = 0; i < ACCEL_CAL_ZERO_SAMPLES; i++) {
        Vec3d r0 = accel1.fetch();
        Vec3d r1 = dual ? accel2.fetch() : r0;
        sumX0 += (float)r0.x();  sumX1 += (float)r1.x();
        sumY0 += (float)r0.y();  sumY1 += (float)r1.y();
        sumZ0 += (float)r0.z();  sumZ1 += (float)r1.z();
        delay(2);  // ~2 ms per sample → 200 samples ≈ 400 ms total
    }

    float n = (float)ACCEL_CAL_ZERO_SAMPLES;
    _cal->captureZero (0, sumX0 / n);
    _cal->captureZero (1, dual ? sumX1 / n : sumX0 / n);
    _cal->captureZeroY(0, sumY0 / n);
    _cal->captureZeroY(1, dual ? sumY1 / n : sumY0 / n);
    _cal->captureZeroZ(0, sumZ0 / n);
    _cal->captureZeroZ(1, dual ? sumZ1 / n : sumZ0 / n);

    Serial.println("[AccelCal] Zero-G capture complete.");
}

