#include "Magnetometer.h"
#include <Wire.h>
#include <math.h>
#include <string.h>

// ─── SpinRateEstimator ────────────────────────────────────────────────────────

void SpinRateEstimator::update(float sample, uint32_t t_us) {
    // Adaptive mean — slow LP filter so it tracks DC offset without
    // following the AC spin signal (τ ≈ 1s at 1000Hz)
    _mean += 0.001f * (sample - _mean);

    bool above = sample > _mean;

    if (above && !_wasAbove) {
        // Rising zero crossing detected.
        // Interpolate precise crossing time between previous and current sample.
        float frac    = (_mean - _prev) / (sample - _prev);
        float t_cross = (float)t_us - frac * 1000.0f;  // assumes 1ms sample period

        // Append to shift buffer — oldest at [0], newest at [_count-1]
        if (_count < MAX_CROSSINGS) {
            _crossings[_count++] = t_cross;
        } else {
            memmove(_crossings, _crossings + 1, (MAX_CROSSINGS - 1) * sizeof(float));
            _crossings[MAX_CROSSINGS - 1] = t_cross;
        }

        if (_count >= 2) _recompute();
    }

    _wasAbove = above;
    _prev     = sample;
}

void SpinRateEstimator::_recompute() {
    // Linearly weighted average of consecutive crossing periods.
    // Period i = crossings[i+1] - crossings[i].
    // Weight = i+1, so the most recent period gets the highest weight.
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;

    for (int i = 0; i < _count - 1; i++) {
        float period = _crossings[i + 1] - _crossings[i];  // µs
        float weight = (float)(i + 1);
        weightedSum += weight * period;
        totalWeight += weight;
    }

    _omega = (2.0f * PI * 1e6f) / (weightedSum / totalWeight);  // rad/s
    _valid = true;
}

// ─── PhaseFitter ──────────────────────────────────────────────────────────────

void PhaseFitter::setOmega(float omega) {
    // If ω changed meaningfully, invalidate so we refit on the next update
    if (fabsf(omega - _omega) > 1.0f) _valid = false;
    _omega = omega;
}

void PhaseFitter::addSample(float mag_x, uint32_t t_us) {
    _x[_head] = mag_x;
    _t[_head] = t_us;
    _head     = (_head + 1) % WINDOW;
    if (_count < WINDOW) _count++;

    // Refit every 10 samples, or immediately if invalidated by an ω change
    if (++_sinceFit >= 10 || (!_valid && _count >= 10)) {
        if (_count >= 10) _refit();
        _sinceFit = 0;
    }
}

void PhaseFitter::_refit() {
    int n = _count;

    // Anchor time to oldest sample in window so cos/sin arguments stay small
    int   i0 = (_head - n + WINDOW) % WINDOW;
    float t0 = (float)_t[i0] * 1e-6f;

    MatrixXf A(n, 3);
    VectorXf b(n);

    for (int i = 0; i < n; i++) {
        int   idx = (_head - n + i + WINDOW) % WINDOW;
        float t   = (float)_t[idx] * 1e-6f - t0;
        float wt  = _omega * t;
        A(i, 0)   = cosf(wt);
        A(i, 1)   = sinf(wt);
        A(i, 2)   = 1.0f;
        b(i)      = _x[idx];
    }

    // Solve mag_x(t) = P·cos(ωt) + Q·sin(ωt) + C for [P, Q, C]
    Vector3f c = A.colPivHouseholderQr().solve(b);
    _phi   = atan2f(-c(1), c(0));
    _valid = true;
}

// ─── MagnetometerTracker ──────────────────────────────────────────────────────

bool MagnetometerTracker::init() {
    if (!_mag.begin()) return false;
    _mag.softReset();
    return true;
}

void MagnetometerTracker::update(uint32_t t_us) {
    uint32_t rawX, rawY, rawZ;
    _mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

    // MMC5983MA: 18-bit output, zero point at 2^17 = 131072, ±8G full scale
    // 16384 counts per Gauss
    float x = ((float)rawX - 131072.0f) / 16384.0f;  // Gauss

    _spin.update(x, t_us);

    if (_spin.isValid()) {
        _phase.setOmega(_spin.getOmega());
        _phase.addSample(x, t_us);

        if (_phase.isValid()) {
            _phiRef = _phase.getPhase();
            _tRef   = t_us;
        }
    }
}

float MagnetometerTracker::getAngle(uint32_t t_us) const {
    if (!isSpinning() || !_phase.isValid()) return 0.0f;
    float dt    = (float)(t_us - _tRef) * 1e-6f;
    float angle = fmodf(_spin.getOmega() * dt + _phiRef, 2.0f * PI);
    return angle < 0.0f ? angle + 2.0f * PI : angle;
}
