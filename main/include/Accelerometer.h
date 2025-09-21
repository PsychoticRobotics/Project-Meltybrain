//
// Created by Atharv Goel on 9/20/25.
//

#ifndef MELTYBRAIN_ACCELEROMETER_H
#define MELTYBRAIN_ACCELEROMETER_H

#include "../lib/Eigen/Core"
#include "../lib/L1S331/LIS331.h"

using namespace Eigen;

class Accelerometer {
public:
    Accelerometer() = default;
    bool init(uint8_t address);

    Vector3d fetch();

private:
    LIS331 base;
};

#endif //MELTYBRAIN_ACCELEROMETER_H
