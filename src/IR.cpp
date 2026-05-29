#include "IR.h"
#include <math.h>

// ─── IRDetector: static ISR dispatch ─────────────────────────────────────────
//
// attachInterrupt() on Teensy 4.x takes a raw void(*)() function pointer, so
// C++ lambdas with captures cannot be used directly.
//
// Solution: a static table of instance pointers + one thin static wrapper per
// slot.  init() finds the first empty slot, stores `this`, and attaches the
// corresponding wrapper.  The wrapper calls _handleInterrupt() on the instance.
//
// If you ever need a fourth concurrent IRDetector, increase
// IRDETECTOR_MAX_INSTANCES in IR.h and add _irISR3 / slot 3 below.

static IRDetector* _irInstances[IRDETECTOR_MAX_INSTANCES] = {};

static void _irISR0() { if (_irInstances[0]) _irInstances[0]->_handleInterrupt(); }
static void _irISR1() { if (_irInstances[1]) _irInstances[1]->_handleInterrupt(); }
static void _irISR2() { if (_irInstances[2]) _irInstances[2]->_handleInterrupt(); }

// Indexed function-pointer table — init() picks the right one automatically.
static void (* const _irISRTable[IRDETECTOR_MAX_INSTANCES])() = {
    _irISR0, _irISR1, _irISR2
};


// ═══════════════════════════════════════════════════════════════════════════════
// IRDetector
// ═══════════════════════════════════════════════════════════════════════════════

void IRDetector::init(uint8_t pin) {
    _pin = pin;

    // TSOP outputs are open-collector with a built-in pull-up.  INPUT_PULLUP
    // adds a second pull-up on the Teensy side, giving a clean HIGH when no
    // modulated signal is present.
    pinMode(_pin, INPUT_PULLUP);

    // Register this instance in the first empty slot and attach its ISR.
    for (int i = 0; i < IRDETECTOR_MAX_INSTANCES; i++) {
        if (_irInstances[i] == nullptr) {
            _irInstances[i] = this;
            // CHANGE mode fires on both edges so we capture the full pulse
            // width, not just the arrival moment.
            attachInterrupt(digitalPinToInterrupt(_pin), _irISRTable[i], CHANGE);
            return;
        }
    }

    // All slots taken — increase IRDETECTOR_MAX_INSTANCES and add a wrapper.
    Serial.println("[IRDetector] ERROR: all ISR slots are in use.");
}

void IRDetector::_handleInterrupt() {
    // digitalReadFast is an inline register read on Teensy 4.x — safe inside
    // an ISR and much cheaper than the generic digitalRead().
    if (digitalReadFast(_pin) == LOW) {
        // ── Falling edge ─────────────────────────────────────────────────────
        // The TSOP just started detecting the modulated carrier — the robot's
        // sensor has swept into alignment with the emitter.
        _fallTime  = micros();
        _detecting = true;
        _pulseDone = false;  // discard any unread previous pulse
    } else {
        // ── Rising edge ───────────────────────────────────────────────────────
        // The TSOP stopped detecting — the robot has swept past the emitter.
        // Only record a complete pulse if we also caught the falling edge;
        // this guards against a spurious rising-edge interrupt during init().
        if (_detecting) {
            _riseTime  = micros();
            _detecting = false;
            _pulseDone = true;   // signal to main loop: data is ready
        }
    }
}

void IRDetector::clearPulse() {
    // Disable interrupts briefly so we can atomically clear all pulse fields.
    // Prevents a race where a new ISR fires between the caller's hasPulse()
    // check and the clearing of _pulseDone.
    noInterrupts();
    _pulseDone = false;
    _fallTime  = 0;
    _riseTime  = 0;
    interrupts();
}


// ═══════════════════════════════════════════════════════════════════════════════
// IRBeaconTracker
// ═══════════════════════════════════════════════════════════════════════════════

void IRBeaconTracker::init(uint8_t beaconAPin, uint8_t beaconBPin) {
    _detA.init(beaconAPin);
    _detB.init(beaconBPin);
}

void IRBeaconTracker::update(uint32_t t_us, float currentAngle, float omega) {
    // Angle measurements are unreliable at very low spin rates because the
    // pulse is wide relative to the angle swept, and back-interpolation error
    // grows.  Skip corrections — but still record detections for calibration.
    bool angularlyReliable = (omega >= IR_MIN_OMEGA);

    // ── Beacon A ──────────────────────────────────────────────────────────────
    if (_detA.hasPulse()) {
        // Back-interpolate: find the angle the robot had at the midpoint of the
        // pulse, not at the current loop iteration (which may be several
        // microseconds later).
        float angle = _backInterpolate(currentAngle, omega, t_us, _detA.getMidTime());

        if (_calibrateA) {
            // Calibration: treat this detection as ground truth and record the
            // current angle as the beacon's reference phase.  Future detections
            // will be compared against it.
            _phaseA     = angle;
            _calibrateA = false;
            Serial.print("[IR] Beacon A phase calibrated: ");
            Serial.print(angle, 4);
            Serial.println(" rad");
        } else if (angularlyReliable) {
            // Normal operation: compute the signed shortest-arc error between
            // where we measured the beacon and where we expect it.
            float error = _wrapError(_phaseA - angle);

            // Reject large errors as probable false positives.  A real beacon
            // detection should land within IR_MAX_VALID_ERROR of the expected
            // phase once the heading has settled.
            if (fabsf(error) <= IR_MAX_VALID_ERROR) {
                _headingError  = error;
                _hasHeadingFix = true;
            }
        }

        _lastAngleA  = angle;
        _lastSeenUsA = t_us;
        _seenA       = true;
        _detA.clearPulse();
    }

    // ── Beacon B ──────────────────────────────────────────────────────────────
    if (_detB.hasPulse()) {
        float angle = _backInterpolate(currentAngle, omega, t_us, _detB.getMidTime());

        if (_calibrateB) {
            _phaseB     = angle;
            _calibrateB = false;
            Serial.print("[IR] Beacon B phase calibrated: ");
            Serial.print(angle, 4);
            Serial.println(" rad");
        } else if (angularlyReliable) {
            float error = _wrapError(_phaseB - angle);

            if (fabsf(error) <= IR_MAX_VALID_ERROR) {
                // Only overwrite with Beacon B's fix if Beacon A hasn't already
                // provided one this loop iteration — avoid double-correcting in a
                // single update() call.
                if (!_hasHeadingFix) {
                    _headingError  = error;
                    _hasHeadingFix = true;
                }
            }
        }

        _lastAngleB  = angle;
        _lastSeenUsB = t_us;
        _seenB       = true;
        _detB.clearPulse();
    }

    // ── Localization ──────────────────────────────────────────────────────────
    // Triangulate position when:
    //   1. Beacon world positions have been configured.
    //   2. Both beacons have been seen within the last IR_STALE_US microseconds.
    //   3. The spin rate is high enough for reliable angle measurements.
    if (_positionsSet && angularlyReliable && _seenA && _seenB) {
        bool aFresh = (t_us - _lastSeenUsA) < IR_STALE_US;
        bool bFresh = (t_us - _lastSeenUsB) < IR_STALE_US;

        if (aFresh && bFresh) {
            _triangulate(_lastAngleA, _lastAngleB);
        }
    }
}

void IRBeaconTracker::calibrate() {
    _calibrateA = true;
    _calibrateB = true;
    Serial.println("[IR] Calibration armed — spin past each beacon to set phases.");
}

void IRBeaconTracker::setBeaconPositions(float ax, float ay, float bx, float by) {
    _ax = ax;  _ay = ay;
    _bx = bx;  _by = by;
    _positionsSet = true;
}

// ── Private helpers ───────────────────────────────────────────────────────────

float IRBeaconTracker::_backInterpolate(float currentAngle, float omega,
                                         uint32_t t_now_us,
                                         uint32_t t_past_us) const {
    // The robot's angle at time t_past_us equals its angle NOW minus however
    // much it has rotated between t_past_us and t_now_us.
    //
    // elapsed_s = (t_now − t_past) / 1e6   [seconds]
    // angle_at_past = currentAngle − omega × elapsed_s
    //
    // Guard against negative elapsed time, which can happen at micros()
    // wraparound (every ~71 minutes) — extremely unlikely to matter in practice,
    // but defensive programming costs nothing here.
    float elapsed_s = (float)(t_now_us - t_past_us) * 1e-6f;
    if (elapsed_s < 0.0f) elapsed_s = 0.0f;

    return _wrapAngle(currentAngle - omega * elapsed_s);
}

float IRBeaconTracker::_wrapAngle(float a) const {
    a = fmodf(a, 2.0f * PI);
    if (a < 0.0f) a += 2.0f * PI;
    return a;
}

float IRBeaconTracker::_wrapError(float e) const {
    // Wrap to [−π, π] so the correction always takes the shortest arc.
    return fmodf(e + 3.0f * PI, 2.0f * PI) - PI;
}

bool IRBeaconTracker::_triangulate(float phiA, float phiB) {
    // ── Ray-intersection triangulation ───────────────────────────────────────
    //
    // At the moment beacon A was detected, the robot's heading was phiA.
    // "Heading phiA" means the robot's sensor was pointing in direction phiA,
    // so beacon A lies in that direction from the robot:
    //
    //     Ax = rx + rA·cos(phiA)
    //     Ay = ry + rA·sin(phiA)
    //
    // Similarly for beacon B:
    //
    //     Bx = rx + rB·cos(phiB)
    //     By = ry + rB·sin(phiB)
    //
    // Rearranging as a 2×2 linear system [rA, rB unknown]:
    //
    //     rA·cos(phiA) − rB·cos(phiB) = Ax − Bx
    //     rA·sin(phiA) − rB·sin(phiB) = Ay − By
    //
    // Solving with Cramer's rule:
    //
    //     det = sin(phiA − phiB)
    //     rA  = [ −(Ax−Bx)·sin(phiB) + cos(phiB)·(Ay−By) ] / det
    //
    //     rx = Ax − rA·cos(phiA)
    //     ry = Ay − rA·sin(phiA)
    // ─────────────────────────────────────────────────────────────────────────

    float det = sinf(phiA - phiB);

    // If |det| is near zero the two beacons appear almost collinear from the
    // robot's position — the intersection is at near-infinity and numerically
    // unreliable.  Skip this update and wait until the robot moves.
    // Threshold 0.05 ≈ 3° of angular separation.
    if (fabsf(det) < 0.05f) return false;

    float dX = _ax - _bx;
    float dY = _ay - _by;

    float rA = (-dX * sinf(phiB) + cosf(phiB) * dY) / det;

    // A negative rA means the beacon is "behind" the detected heading — this
    // indicates misconfigured phases or a spurious detection.  Discard.
    if (rA < 0.0f) return false;

    _posX        = _ax - rA * cosf(phiA);
    _posY        = _ay - rA * sinf(phiA);
    _hasPosition = true;
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════════
// IRSweep
// ═══════════════════════════════════════════════════════════════════════════════

void IRSweep::init(uint8_t emitterPin, uint8_t receiverPin) {
    _emitterPin = emitterPin;

    // Start with emitter off.
    pinMode(_emitterPin, OUTPUT);
    digitalWrite(_emitterPin, LOW);

    // Pre-configure the FlexPWM timer at 36 kHz so enable() just calls
    // analogWrite() — no frequency reprogramming overhead at runtime.
    // 36 kHz was chosen to match TSOP38436 and differ from the 38/40 kHz
    // beacon channels, preventing any cross-channel false triggers.
    analogWriteFrequency(_emitterPin, IR_SWEEP_FREQ);

    _receiver.init(receiverPin);
}

void IRSweep::enable() {
    if (!_enabled) {
        // 50 % duty cycle (128 / 255) maximises LED pulse energy while keeping
        // average current within safe GPIO source limits (~10 mA).
        // For longer detection range, drive the LED through a transistor and
        // route the base/gate from this pin rather than sourcing through it.
        analogWrite(_emitterPin, 128);
        _enabled = true;
    }
}

void IRSweep::disable() {
    if (_enabled) {
        analogWrite(_emitterPin, 0);
        _enabled = false;
    }
}

void IRSweep::update(uint32_t t_us, float currentAngle, float omega) {
    if (!_enabled)              return;
    if (!_receiver.hasPulse())  return;

    // Back-interpolate to find the arena heading at the midpoint of the pulse.
    float elapsed_s = (float)(t_us - _receiver.getMidTime()) * 1e-6f;
    if (elapsed_s < 0.0f) elapsed_s = 0.0f;

    float hitAngle = _wrapAngle(currentAngle - omega * elapsed_s);

    _receiver.clearPulse();

    // ── Filter: known beacon directions ───────────────────────────────────────
    // If the hit falls within IR_SWEEP_FILTER_RAD of a known beacon, it is
    // almost certainly a reflection off the beacon emitter itself (or the
    // arena structure near it) — not an opponent.  Skip it.
    if (_filterBeacons && _isKnownBeacon(hitAngle)) return;

    // ── Filter: duplicate hits from ISR jitter ────────────────────────────────
    // A single physical reflector can cause multiple ISR firings within a very
    // short angle window as the TSOP re-triggers.  Only store a hit if it is
    // IR_SWEEP_MIN_HIT_SPACING radians away from the previous one.
    if (_hitCount > 0) {
        float prev = _hitAngles[_hitCount - 1];
        float diff = fabsf(hitAngle - prev);
        if (diff > PI) diff = 2.0f * PI - diff;  // take the shorter arc
        if (diff < IR_SWEEP_MIN_HIT_SPACING) return;
    }

    // ── Store the hit ─────────────────────────────────────────────────────────
    if (_hitCount < IR_SWEEP_MAX_HITS) {
        // Buffer has room — just append.
        _hitAngles[_hitCount++] = hitAngle;
    } else {
        // Buffer full — shift everything down by one and append at the end,
        // effectively dropping the oldest hit.  This keeps the most recent
        // detections available even in a target-dense environment.
        for (int i = 0; i < IR_SWEEP_MAX_HITS - 1; i++) {
            _hitAngles[i] = _hitAngles[i + 1];
        }
        _hitAngles[IR_SWEEP_MAX_HITS - 1] = hitAngle;
        // _hitCount stays at IR_SWEEP_MAX_HITS.
    }
}

float IRSweep::getHitAngle(int index) const {
    if (index < 0 || index >= _hitCount) return 0.0f;
    return _hitAngles[index];
}

void IRSweep::clearHits() {
    _hitCount = 0;
}

bool IRSweep::_isKnownBeacon(float angle) const {
    // Compute the shortest angular distance between `angle` and each beacon
    // phase, and return true if either is within the filter window.
    auto diff = [](float a, float b) -> float {
        float d = fabsf(a - b);
        return (d > PI) ? (2.0f * PI - d) : d;
    };
    return diff(angle, _phaseA) < IR_SWEEP_FILTER_RAD
        || diff(angle, _phaseB) < IR_SWEEP_FILTER_RAD;
}

float IRSweep::_wrapAngle(float a) const {
    a = fmodf(a, 2.0f * PI);
    if (a < 0.0f) a += 2.0f * PI;
    return a;
}
