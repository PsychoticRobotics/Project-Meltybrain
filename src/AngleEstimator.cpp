#include "AngleEstimator.h"
#include <math.h>

AngleEstimator::AngleEstimator(AccelerometerManager& accel,
                               MagnetometerTracker&  mag,
                               float y1,
                               float y2,
                               float fusionGain)
    : _accel(&accel), _mag(&mag),
      _y1(y1), _y2(y2),
      _fusionGain(fusionGain),
      _r1(y1), _r2(y2)   // initial best-guess radii equal geometric distances
{}

void AngleEstimator::update(uint32_t t_us) {
    if (_firstUpdate) {
        _lastTime    = t_us;
        _firstUpdate = false;
        return;
    }

    float dt = (float)(t_us - _lastTime) * 1e-6f;  // seconds
    _lastTime = t_us;

    // ── ω estimation ──────────────────────────────────────────────────────────
    if (_accel->isDual()) {
        // ── Two-sensor differential formula ──────────────────────────────────
        //
        // Both accelerometers lie on the body y-axis (the axis connecting them):
        //   accel1 sits at position +_y1 from the robot's geometric centre
        //   accel2 sits at position −_y2 from the robot's geometric centre
        //
        // The spinning centre is offset (Cx, Cy) from the geometric centre.
        // The centripetal acceleration at each sensor has a component along the
        // separation (y) axis equal to:
        //
        //   ay1 = ω² × (Cy − _y1)   ← negative if Cy < y1 (top sensor)
        //   ay2 = ω² × (Cy + _y2)   ← positive (bottom sensor, further from centre)
        //
        // Subtracting eliminates Cy:
        //   ay2 − ay1 = ω² × (_y1 + _y2)   →   ω² = (ay2 − ay1) / (_y1 + _y2)
        //
        // AXIS ASSUMPTION:  fetch().y() maps to the separation axis,
        //                   fetch().x() maps to the perpendicular in-plane axis.
        // If your physical sensor mounting uses different axes, swap .y() / .x() here.

        Vec3d raw1 = _accel->fetchXYZ1();   // must call accelerometers.refresh() first
        Vec3d raw2 = _accel->fetchXYZ2();

        float ay1 = (float)raw1.y();
        float ay2 = (float)raw2.y();
        float ax  = (float)(raw1.x() + raw2.x()) * 0.5f;   // average; should be equal

        float omegaSq = (ay2 - ay1) / (_y1 + _y2);

        if (omegaSq > 0.0f) {
            // Gentle low-pass on ω to smooth sensor noise.
            float omegaRaw = sqrtf(omegaSq);
            _omega = 0.8f * _omega + 0.2f * omegaRaw;

            // Spinning-centre geometry (recomputed every loop — Cy shifts with thrust).
            float w2 = _omega * _omega;
            _cx = ax  / w2;                      // x-offset: static imbalance
            _cy = ay1 / w2 + _y1;                // y-offset: moves with motor power
            _r1 = sqrtf(_cx*_cx + (_y1 - _cy)*(_y1 - _cy));
            _r2 = sqrtf(_cx*_cx + (_y2 + _cy)*(_y2 + _cy));
        }
        // omegaSq ≤ 0 → robot is not yet spinning; hold the previous _omega,
        // _cx, _cy, _r1, _r2 until a valid reading arrives.

    } else {
        // ── Single-sensor fallback ────────────────────────────────────────────
        // Use the NTU normal component (centripetal direction) and treat _y1 as
        // the fixed spin radius.  This matches the original behaviour.
        float normalAccel = (float)_accel->fetchNTU().x();
        _filteredAccel    = 0.9f * _filteredAccel + 0.1f * normalAccel;
        _omega = sqrtf(fmaxf(_filteredAccel / _y1, 0.0f));
        _cx = 0.0f;
        _cy = 0.0f;
        _r1 = _y1;
        _r2 = _y2;
    }

    _angle += _omega * dt;

    // ── Magnetometer: slowly correct accumulated drift ─────────────────────────
    // mag.update() must be called before this
    if (_mag->isAngleValid()) {
        float magAngle = _mag->getAngle(t_us);
        // Wrap error to [-π, π] so correction always takes the short way around.
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
