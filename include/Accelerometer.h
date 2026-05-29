#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/Eigen/Core"
#include "../lib/L1S331/LIS331.h"
#include "AccelCalibration.h"
#include "Config.h"
#include <SD.h>

using namespace Eigen;

class Accelerometer {
public:
    bool initialized = false;

    void init(int addr);
    Vector3d fetch();
    void setAdjustment(Vector3d offset, Vector3d scale);
private:
    LIS331 base;
    Vector3d _offset{0, 0, 0};
    Vector3d _scale{1, 1, 1};
};

class AccelerometerManager {
public:
    void init(int addr1, int addr2);
    void init(int addr);
    void setAdjustments(Vector3d offset1, Vector3d scale1,
                        Vector3d offset2, Vector3d scale2);

    /**
     * Attach a runtime calibration manager.  Once attached, refresh() applies
     * calibration corrections to the separation axis (y) of each sensor's
     * individual cache before computing the average.
     * Call from setup() after accelCal.load().
     */
    void attachCalibration(AccelCalibrationManager* cal) { _cal = cal; }

    /** Read sensors once and cache — call at the start of each loop. */
    void refresh();

    /** True when both accelerometers are present and initialised. */
    bool isDual() const { return accel1.initialized && accel2.initialized; }

    // ── Average-of-both (or single) readings ──────────────────────────────
    Vector3d fetchXYZ();   // calibration-corrected average across sensors
    Vector3d fetchNTU();   // averaged, then rotated into Normal-Tangential-Up frame

    // ── Per-sensor readings (for differential centre-of-rotation calc) ────
    // These return the individual cached readings with calibration already applied.
    // If only one sensor is fitted, fetchXYZ2 mirrors fetchXYZ1.
    Vector3d fetchXYZ1();
    Vector3d fetchXYZ2();

    /**
     * Zero-G offset capture (blocking, ~400 ms).
     * Reads ACCEL_CAL_ZERO_SAMPLES raw samples from each sensor's separation
     * axis (y), averages them, and stores the result via cal.captureZero().
     * The robot MUST be completely stationary during this call.
     * The calibration manager must already be attached (attachCalibration).
     */
    void captureZeroG();

    void log(File& logger, uint32_t time);

private:
    Accelerometer accel1;
    Accelerometer accel2;

    AccelCalibrationManager* _cal = nullptr;  // null = no calibration

    Vector3d _cache {0, 0, 0};  // averaged (or single) — used by fetchXYZ / fetchNTU
    Vector3d _cache1{0, 0, 0};  // accel1 individual reading (calibration applied)
    Vector3d _cache2{0, 0, 0};  // accel2 individual reading (mirrors _cache1 if not fitted)
};

#endif //MELTYBRAIN_ACCELEROMETER_H
