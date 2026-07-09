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
        // Physical layout:
        //   Both sensors have identical orientation: x = right, y = forward, z = up.
        //   They sit on the left-right axis of the robot, 22.6 mm chip-to-chip.
        //   The midpoint is the geometric centre.
        //     accel1 (0x18): one side,  _y1 = 11.3 mm from centre
        //     accel2 (0x19): other side, _y2 = 11.3 mm from centre
        //   (verify in hardware which address is left vs right)
        //
        // The separation axis is sensor-x (left-right).  Centripetal acceleration
        // at each sensor has a component along that axis proportional to distance
        // from the spin centre (Cx) in that direction:
        //
        //   ax1 = −ω² × (Cx − _y1)   (sign depends on which side is accel1)
        //   ax2 = −ω² × (Cx + _y2)
        //
        // Subtracting eliminates Cx:
        //   ax1 − ax2 = ω² × (_y1 + _y2)   →   ω² = (ax1 − ax2) / (_y1 + _y2)
        //
        // If ω² comes out negative, the sign convention is flipped (accel1 is on
        // the opposite side from assumed); swap ax1/ax2 or negate _y1/_y2.
        //
        // The perpendicular in-plane axis is sensor-y (forward/back).
        // Its average across both sensors gives the tangential / forward acceleration.

        Vec3d raw1 = _accel->fetchXYZ1();   // must call accelerometers.refresh() first
        Vec3d raw2 = _accel->fetchXYZ2();

        // Separation axis (left-right = sensor x):
        float ax1 = (float)raw1.x();
        float ax2 = (float)raw2.x();
        // Perpendicular in-plane axis (forward = sensor y); should be equal for both:
        float ay  = (float)(raw1.y() + raw2.y()) * 0.5f;

        float omegaSq = (ax1 - ax2) / (_y1 + _y2);

        if (omegaSq > 0.0f) {
            // Gentle low-pass on ω to smooth sensor noise.
            float omegaRaw = sqrtf(omegaSq);
            _omega = 0.8f * _omega + 0.2f * omegaRaw;

            // Spinning-centre geometry (recomputed every loop — Cx shifts with thrust).
            float w2 = _omega * _omega;
            _cx = ay  / w2;                      // offset along forward axis: static imbalance
            _cy = ax1 / w2 + _y1;               // offset along separation axis: moves with thrust
            _r1 = sqrtf(_cx*_cx + (_y1 - _cy)*(_y1 - _cy));
            _r2 = sqrtf(_cx*_cx + (_y2 + _cy)*(_y2 + _cy));
        }
        // omegaSq ≤ 0 → robot is not yet spinning (or sign convention is flipped);
        // hold the previous _omega, _cx, _cy, _r1, _r2 until a valid reading arrives.

    } else {
        // ── Single-sensor fallback ────────────────────────────────────────────
        // Use the NTU normal component (centripetal direction) and treat _y1 as
        // the fixed spin radius (11.3 mm from centre to sensor chip).
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
