#ifndef MELTYBRAIN_MAGNETOMETER_H
#define MELTYBRAIN_MAGNETOMETER_H

// Usage:
//
//   MagnetometerTracker mag;
//
//   setup():
//     mag.init();
//
//   loop():
//     mag.update(currentTime);              // call after accelerometers.refresh()
//     float angle = mag.getAngle(currentTime);
//     float rpm   = mag.getRPM();

#include <Arduino.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
// Arduino.h defines B0/B1/min/max macros that collide with Eigen internals.
#ifdef B1
#undef B1
#endif
#ifdef B0
#undef B0
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include "../lib/Eigen/Dense"

using namespace Eigen;

// ─── SpinRateEstimator ────────────────────────────────────────────────────────
// Detects rising zero crossings on a single magnetometer axis and estimates
// angular velocity using a linearly-weighted average of recent crossing periods.
// More recent periods are weighted higher, smoothing jitter without sacrificing
// responsiveness.

class SpinRateEstimator {
public:
    static constexpr int MAX_CROSSINGS = 12;

    void  update(float sample, uint32_t t_us);
    float getOmega() const { return _omega; }                          // rad/s
    float getRPM()   const { return _omega * (60.0f / (2.0f * PI)); }
    bool  isValid()  const { return _valid; }

private:
    float _crossings[MAX_CROSSINGS] = {};  // µs timestamps, oldest at [0]
    int   _count    = 0;
    float _mean     = 0.0f;                // adaptive DC offset
    float _prev     = 0.0f;
    bool  _wasAbove = false;
    float _omega    = 0.0f;
    bool  _valid    = false;

    void _recompute();
};

// ─── PhaseFitter ──────────────────────────────────────────────────────────────
// Fits mag_x(t) = P·cos(ωt) + Q·sin(ωt) + C over a rolling sample window
// using linear least squares. Returns phase φ = atan2(-Q, P).
// ω must be supplied externally from SpinRateEstimator.

class PhaseFitter {
public:
    static constexpr int WINDOW = 200;  // ~200ms at 1000Hz

    void  addSample(float mag_x, uint32_t t_us);
    void  setOmega(float omega);
    float getPhase() const { return _phi; }
    bool  isValid()  const { return _valid; }

private:
    float    _x[WINDOW]  = {};
    uint32_t _t[WINDOW]  = {};
    int      _head       = 0;
    int      _count      = 0;
    float    _omega      = 0.0f;
    float    _phi        = 0.0f;
    bool     _valid      = false;
    int      _sinceFit   = 0;

    void _refit();
};

// ─── MagnetometerTracker ──────────────────────────────────────────────────────
// Top-level class. Reads the MMC5983MA, pipes the X axis through
// SpinRateEstimator → PhaseFitter, and exposes spin rate and angle.

class MagnetometerTracker {
public:
    bool  init();                             // returns false if sensor not found
    void  update(uint32_t t_us);              // call once per loop
    float getOmega()              const { return _spin.getOmega(); }
    float getRPM()                const { return _spin.getRPM(); }
    float getAngle(uint32_t t_us) const;      // extrapolated angle in [0, 2π]
    bool  isSpinning()            const { return _spin.isValid(); }
    bool  isAngleValid()          const { return _spin.isValid() && _phase.isValid(); }

private:
    SFE_MMC5983MA     _mag;
    SpinRateEstimator _spin;
    PhaseFitter       _phase;
    float             _phiRef = 0.0f;
    uint32_t          _tRef   = 0;
};

#endif //MELTYBRAIN_MAGNETOMETER_H
