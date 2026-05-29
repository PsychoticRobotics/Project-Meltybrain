#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/Eigen/Core"
#include "../lib/L1S331/LIS331.h"
#include "Config.h"
#include <SD.h> // Add this include

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

    /** Read sensors once and cache — call at the start of each loop. */
    void refresh();

    /** True when both accelerometers are present and initialised. */
    bool isDual() const { return accel1.initialized && accel2.initialized; }

    // ── Average-of-both (or single) readings ──────────────────────────────
    Vector3d fetchXYZ();   // raw counts → g, averaged across sensors
    Vector3d fetchNTU();   // averaged, then rotated into Normal-Tangential-Up frame

    // ── Per-sensor readings (for differential centre-of-rotation calc) ────
    // fetchXYZ1/2 return the individual cached raw readings (in g).
    // If only one sensor is fitted, fetchXYZ2 mirrors fetchXYZ1.
    Vector3d fetchXYZ1();
    Vector3d fetchXYZ2();

    void log(File& logger, uint32_t time);

private:
    Accelerometer accel1;
    Accelerometer accel2;

    Vector3d _cache {0, 0, 0};  // averaged (or single) — used by fetchXYZ / fetchNTU
    Vector3d _cache1{0, 0, 0};  // accel1 individual reading
    Vector3d _cache2{0, 0, 0};  // accel2 individual reading (mirrors _cache1 if not fitted)
};

#endif //MELTYBRAIN_ACCELEROMETER_H
