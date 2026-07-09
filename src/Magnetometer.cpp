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
    // Fit mag_x(t) = P·cos(ωt) + Q·sin(ωt) + C for [P, Q, C].
    //
    // Solving the 3×3 normal equations directly avoids pulling in Eigen just
    // for a QR decomposition on a rank-3 problem. Sums are accumulated over
    // the sample window; Cramer's rule closes out the tiny linear system.
    //
    //   Let a_i = cos(ω·τ_i),  b_i = sin(ω·τ_i),  c_i = 1
    //   Normal-equation matrix M = Aᵀ A:
    //     M = [ Σa²   Σab   Σa  ]
    //         [ Σab   Σb²   Σb  ]
    //         [ Σa    Σb    n   ]
    //   RHS   r = Aᵀ y:
    //     r = [ Σa·y ; Σb·y ; Σy ]
    //   Solve M · [P Q C]ᵀ = r via Cramer.

    int n = _count;

    // Anchor time to the oldest sample so ω·τ stays small.
    int   i0 = (_head - n + WINDOW) % WINDOW;
    float t0 = (float)_t[i0] * 1e-6f;

    float Saa = 0.0f, Sab = 0.0f, Sa = 0.0f;
    float Sbb = 0.0f, Sb = 0.0f;
    float Say = 0.0f, Sby = 0.0f, Sy = 0.0f;

    for (int i = 0; i < n; i++) {
        int   idx = (_head - n + i + WINDOW) % WINDOW;
        float t   = (float)_t[idx] * 1e-6f - t0;
        float wt  = _omega * t;
        float a   = cosf(wt);
        float b   = sinf(wt);
        float y   = _x[idx];

        Saa += a * a;
        Sab += a * b;
        Sa  += a;
        Sbb += b * b;
        Sb  += b;
        Say += a * y;
        Sby += b * y;
        Sy  += y;
    }

    // 3×3 determinant of M via cofactor expansion along row 0.
    float m00 = Sbb * (float)n - Sb  * Sb;
    float m01 = Sab * (float)n - Sb  * Sa;
    float m02 = Sab * Sb        - Sbb * Sa;

    float det = Saa * m00 - Sab * m01 + Sa * m02;

    // Rank-deficient / ill-conditioned window — skip this fit, keep last _phi.
    if (fabsf(det) < 1e-12f) {
        _valid = false;
        return;
    }
    float invDet = 1.0f / det;

    // Cramer's rule — replace each column of M with r and take the determinant.
    // Column 0 replaced (solves for P):
    float detP = Say * m00
               - Sab * (Sby * (float)n - Sb * Sy)
               + Sa  * (Sby * Sb        - Sbb * Sy);

    // Column 1 replaced (solves for Q):
    float detQ = Saa * (Sby * (float)n - Sb * Sy)
               - Say * m01
               + Sa  * (Sab * Sy        - Say * Sb);

    float P = detP * invDet;
    float Q = detQ * invDet;
    // C is not needed downstream — atan2 uses only P, Q — so we skip computing it.

    _phi   = atan2f(-Q, P);
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
