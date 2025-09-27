//
// Created by Atharv Goel on 9/20/25.
// Updated last by Atharv Goel on 9/27/25.
//

#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/Eigen/Core"
#include "../lib/L1S331/LIS331.h"

using namespace Eigen;

class Accelerometer {
public:
    bool initialized = false;

    bool init(uint8_t address);
    Vector3d fetch();

private:
    LIS331 base;
};

class AccelerometerManager {
public:
    Accelerometer accel1;
    Accelerometer accel2;

    bool setup(uint8_t add1, uint8_t add2);
    bool setup(uint8_t add);
    Vector3d fetch();
};

#endif //MELTYBRAIN_ACCELEROMETER_H
