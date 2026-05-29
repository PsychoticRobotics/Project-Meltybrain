#include "AngleEstimator.h"
#include <math.h>

AngleEstimator::AngleEstimator(AccelerometerManager& accel,
                               MagnetometerTracker&  mag,
                               float accelRadius,
                               float fusionGain)
    : _accel(&accel), _mag(&mag),
      _accelRadius(accelRadius), _fusionGain(fusionGain) {}

void AngleEstimator::update(uint32_t t_us) {
    if (_firstUpdate) {
        _lastTime    = t_us;
        _firstUpdate = false;
        return;
    }

    float dt = (float)(t_us - _lastTime) * 1e-6f;  // seconds
    _lastTime = t_us;

    // ── Accelerometer: centripetal acceleration → ω → integrate angle ─────────
    // fetchNTU() returns cached data — accelerometers.refresh() must be called first
    float normalAccel  = (float)_accel->fetchNTU().x();
    _filteredAccel     = 0.9f * _filteredAccel + 0.01f * normalAccel;
    _omega             = sqrtf(fmaxf(_filteredAccel / _accelRadius, 0.0f));
    _angle            += _omega * dt;

    // ── Magnetometer: slowly correct accumulated drift ─────────────────────────
    // mag.update() must be called before this
    if (_mag->isAngleValid()) {
        float magAngle = _mag->getAngle(t_us);
        // Wrap error to [-π, π] so correction always takes the short way around
        float error = fmodf(magAngle - _angle + 3.0f * PI, 2.0f * PI) - PI;
        _angle += _fusionGain * error;
    }

    // Wrap angle to [0, 2π]
    _angle = fmodf(_angle, 2.0f * PI);
    if (_angle < 0.0f) _angle += 2.0f * PI;
}

void AngleEstimator::correctAngle(float delta) {
    // Apply an external angle correction (e.g., from an IR beacon fix) and
    // re-wrap into [0, 2π].  Intentionally separate from update() so the IR
    // system can be added or removed in main.cpp without touching this class.
    _angle += delta;
    _angle = fmodf(_angle, 2.0f * PI);
    if (_angle < 0.0f) _angle += 2.0f * PI;
}
