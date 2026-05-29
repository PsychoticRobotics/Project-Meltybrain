#ifndef MELTYBRAIN_ANGLE_ESTIMATOR_H
#define MELTYBRAIN_ANGLE_ESTIMATOR_H

// Usage:
//
//   // One accelerometer (default 33 mm radius, single-sensor fallback):
//   AngleEstimator estimator(accelerometers, mag);
//
//   // Two accelerometers (differential ω — no hardcoded radius needed):
//   // y1 = distance from geometric centre to the "top" sensor (metres)
//   // y2 = distance from geometric centre to the "bottom" sensor (metres)
//   AngleEstimator estimator(accelerometers, mag, /*y1=*/0.030f, /*y2=*/0.025f);
//
//   setup():
//     accelerometers.init(addr1, addr2);  // both sensors for dual mode
//
//   loop():
//     accelerometers.refresh();
//     mag.update(currentTime);
//     estimator.update(currentTime);   // must come after both sensors
//     float angle = estimator.getAngle();
//     float omega = estimator.getOmega();
//     // When dual: estimator.getCy() tracks the live spinning-centre y-offset

#include "Accelerometer.h"
#include "Magnetometer.h"

class AngleEstimator {
public:
    // y1, y2     : distance from the robot's geometric centre to each accelerometer
    //              along the separation axis, in metres (both positive scalars).
    //              accel1 is placed at +y1 (e.g. the "top" sensor),
    //              accel2 is placed at −y2 (e.g. the "bottom" sensor).
    //              When only one accelerometer is fitted, y1 is used as the spin
    //              radius for the single-sensor fallback.
    // fusionGain : magnetometer soft-correction weight per update (try 0.005)
    AngleEstimator(AccelerometerManager& accel,
                   MagnetometerTracker&  mag,
                   float y1         = 0.033f,
                   float y2         = 0.033f,
                   float fusionGain = 0.005f);

    void  update(uint32_t t_us);

    float getAngle() const { return _angle; }   // radians, [0, 2π]
    float getOmega() const { return _omega; }   // rad/s

    // Spinning centre offset from geometric centre (metres).
    // Only meaningful when two accelerometers are fitted (isDual).
    // Cy shifts dynamically as motor power changes; Cx reflects static imbalance.
    float getCx() const { return _cx; }  // offset along the perpendicular axis
    float getCy() const { return _cy; }  // offset along the separation axis

    // Effective spin radii to each accelerometer (metres).
    float getR1() const { return _r1; }
    float getR2() const { return _r2; }

    // Apply an external angle correction delta (radians) — for example from an
    // IR beacon fix.  The result is wrapped back into [0, 2π].
    // Typical call:  estimator.correctAngle(IR_SNAP_GAIN * beacons.getHeadingError())
    void  correctAngle(float delta);

private:
    AccelerometerManager* _accel;
    MagnetometerTracker*  _mag;

    float    _y1;            // physical distance from geometric centre to accel1 (m)
    float    _y2;            // physical distance from geometric centre to accel2 (m)
    float    _fusionGain;

    float    _angle         = 0.0f;
    float    _omega         = 0.0f;
    float    _filteredAccel = 0.0f;  // used in single-sensor fallback path only

    // Spinning-centre geometry — updated every loop when two sensors are present.
    float    _cx = 0.0f;     // spin-centre x-offset from geometric centre (m)
    float    _cy = 0.0f;     // spin-centre y-offset (shifts with motor power)
    float    _r1 = 0.033f;   // effective radius to accel1 from spin centre (m)
    float    _r2 = 0.033f;   // effective radius to accel2 from spin centre (m)

    uint32_t _lastTime      = 0;
    bool     _firstUpdate   = true;
};

#endif //MELTYBRAIN_ANGLE_ESTIMATOR_H
