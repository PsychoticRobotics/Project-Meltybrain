#ifndef MELTYBRAIN_ANGLE_ESTIMATOR_H
#define MELTYBRAIN_ANGLE_ESTIMATOR_H

// Usage:
//
//   AngleEstimator estimator(accelerometers, mag);
//
//   setup():
//     (no init needed — sensors are initialised separately)
//
//   loop():
//     accelerometers.refresh();
//     mag.update(currentTime);
//     estimator.update(currentTime);   // must come after both sensors
//     float angle = estimator.getAngle();

#include "Accelerometer.h"
#include "Magnetometer.h"

class AngleEstimator {
public:
    // accelRadius: distance from centre of rotation to accelerometer (metres)
    // fusionGain:  how strongly the magnetometer corrects drift each loop
    //              0 = accelerometer only, higher = faster correction (try 0.005)
    AngleEstimator(AccelerometerManager& accel,
                   MagnetometerTracker&  mag,
                   float accelRadius = 0.033f,
                   float fusionGain  = 0.005f);

    void  update(uint32_t t_us);

    float getAngle() const { return _angle; }   // radians, [0, 2π]
    float getOmega() const { return _omega; }   // rad/s

private:
    AccelerometerManager* _accel;
    MagnetometerTracker*  _mag;

    float    _accelRadius;
    float    _fusionGain;

    float    _angle         = 0.0f;
    float    _omega         = 0.0f;
    float    _filteredAccel = 0.0f;
    uint32_t _lastTime      = 0;
    bool     _firstUpdate   = true;
};

#endif //MELTYBRAIN_ANGLE_ESTIMATOR_H
