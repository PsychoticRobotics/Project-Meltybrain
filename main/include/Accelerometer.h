//
// Created by Atharv Goel on 9/20/25.
// Updated last by Atharv Goel on 9/28/25.
//

#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/Eigen/Core"
#include "../lib/L1S331/LIS331.h"
#include "Config.h"

using namespace Eigen;

class Accelerometer {
public:
    bool initialized = false;

    bool init(int in);
    Vector3d fetch();

private:
    LIS331 base;
};

class AccelerometerManager {
public:
    Accelerometer accel1;
    Accelerometer accel2;

    bool setup(int in1, int in2);
    bool setup(int in);
    Vector3d fetch(Basis basis);
};

#endif //MELTYBRAIN_ACCELEROMETER_H
