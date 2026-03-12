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
    Accelerometer accel1;
    Accelerometer accel2;

    void init(int addr1, int addr2);
    void init(int addr);
    void setAdjustments(Vector3d offset1, Vector3d scale1,
                    Vector3d offset2, Vector3d scale2);
    Vector3d fetchXYZ();
    Vector3d fetchNTU();
    void log(File& logger, uint32_t time);
};

#endif //MELTYBRAIN_ACCELEROMETER_H
