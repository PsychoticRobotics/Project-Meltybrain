#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/L1S331/LIS331.h"
// LIS331.h defines STATUS_REG as a macro — undef it before any other header
// can try to declare a variable with the same name (e.g. SparkFun MMC5983MA).
#ifdef STATUS_REG
#undef STATUS_REG
#endif
#include "AccelCalibration.h"
#include "Config.h"

// Minimal 3-component double vector — replaces Eigen::Vector3d so we don't
// have to pull in Eigen and fight Arduino's B0/B1/F macro collisions.
struct Vec3d {
    double _x = 0.0, _y = 0.0, _z = 0.0;
    Vec3d() = default;
    Vec3d(double x, double y, double z) : _x(x), _y(y), _z(z) {}
    double&       x()       { return _x; }
    double&       y()       { return _y; }
    double&       z()       { return _z; }
    const double& x() const { return _x; }
    const double& y() const { return _y; }
    const double& z() const { return _z; }
    Vec3d operator+(const Vec3d& o) const { return {_x+o._x, _y+o._y, _z+o._z}; }
    Vec3d operator*(double s)       const { return {_x*s,    _y*s,    _z*s};    }
};

class Accelerometer {
public:
    bool initialized = false;

    void init(int addr);
    Vec3d fetch();
    void setAdjustment(Vec3d offset, Vec3d scale);
private:
    LIS331 base;
    Vec3d _offset{0, 0, 0};
    Vec3d _scale{1, 1, 1};
};

class AccelerometerManager {
public:
    void init(int addr1, int addr2);
    void init(int addr);
    void setAdjustments(Vec3d offset1, Vec3d scale1,
                        Vec3d offset2, Vec3d scale2);

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
    Vec3d fetchXYZ();   // calibration-corrected average across sensors

    // ── Per-sensor readings (for differential centre-of-rotation calc) ────
    // These return the individual cached readings with calibration already applied.
    // If only one sensor is fitted, fetchXYZ2 mirrors fetchXYZ1.
    Vec3d fetchXYZ1();
    Vec3d fetchXYZ2();

    /**
     * Zero-G offset capture (blocking, ~400 ms).
     * Reads ACCEL_CAL_ZERO_SAMPLES raw samples from each sensor's separation
     * axis (y), averages them, and stores the result via cal.captureZero().
     * The robot MUST be completely stationary during this call.
     * The calibration manager must already be attached (attachCalibration).
     */
    void captureZeroG();

private:
    Accelerometer accel1;
    Accelerometer accel2;

    AccelCalibrationManager* _cal = nullptr;  // null = no calibration

    Vec3d _cache {0, 0, 0};  // averaged (or single) — used by fetchXYZ
    Vec3d _cache1{0, 0, 0};  // accel1 individual reading (calibration applied)
    Vec3d _cache2{0, 0, 0};  // accel2 individual reading (mirrors _cache1 if not fitted)
};

#endif //MELTYBRAIN_ACCELEROMETER_H
