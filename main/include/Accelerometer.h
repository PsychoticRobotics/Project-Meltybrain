#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/Eigen/Core"
#include "../lib/L1S331/LIS331.h"
#include "Config.h"

using namespace Eigen;

class Accelerometer {
public:
    bool initialized = false;

    bool init(int addr);
    Vector3d fetch();

private:
    LIS331 base;
};

class AccelerometerManager {
public:
    Accelerometer accel1;
    Accelerometer accel2;

    bool init(int addr1, int addr2);
    bool init(int addr);
    Vector3d fetchXYZ();
    Vector3d fetchNTU();
};

#endif //MELTYBRAIN_ACCELEROMETER_H
