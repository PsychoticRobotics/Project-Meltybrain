#ifndef MELTYBRAIN_IR_H
#define MELTYBRAIN_IR_H

// ─── IR: Custom rotating scanner ("LiDAR") for meltybrain ────────────────────
//
// The robot's own spin is the scanning mechanism — sensors mounted on the body
// sweep 360° with every revolution, turning a fixed sensor into a rotating
// scanner with no moving parts.
//
// ─── Goals ───────────────────────────────────────────────────────────────────
//
//   1. Heading reference   – absolute angle correction every revolution.
//                            Supplements or replaces the magnetometer.
//   2. Localization        – 2-D position estimate from two beacon bearings.
//   3. Opponent detection  – passive sweep: reflections that don't match known
//                            beacon angles are reported as unknown targets.
//
// ─── Hardware ─────────────────────────────────────────────────────────────────
//
//   Outside the arena (fixed, known positions):
//     Beacon A: 850 nm IR LED + 38 kHz oscillator (555 timer or Arduino Nano
//               running  tone(pin, 38000) ).
//     Beacon B: same, but at 40 kHz — hardware-distinguished with no decoding.
//
//   On the robot (all pointing radially outward):
//     TSOP38438 → IR_BEACON_A_PIN   38 kHz receiver, detects Beacon A
//     TSOP38440 → IR_BEACON_B_PIN   40 kHz receiver, detects Beacon B
//     TSOP38436 → IR_SWEEP_RX_PIN   36 kHz receiver, detects sweep reflections
//     IR LED    ← IR_SWEEP_TX_PIN   36 kHz emitter for active sweep
//
//   TSOP pinout (flat side toward you):  [OUT] [GND] [VCC]
//   Connect VCC → 3.3 V, GND → GND, OUT → Teensy signal pin.
//   The output is active-LOW: HIGH = no signal, LOW = modulated burst detected.
//
//   IR LED: anode → 47 Ω resistor → Teensy GPIO pin, cathode → GND.
//   At 10 mA (direct GPIO) range is ~0.5 m. For longer range drive the LED
//   through a transistor (2N2222 / IRLZ44N) to reach 50–100 mA.
//
//   Shell material: polycarbonate transmits 850 nm IR at ~85 % — no holes
//   needed. Metal shells require a ~4 mm hole per sensor. Mount a thin black
//   foam or tape baffle between the sweep emitter and sweep receiver to prevent
//   the emitter reflecting off the inside of the shell back to the receiver.
//
// ─── Classes ──────────────────────────────────────────────────────────────────
//
//   IRDetector       – interrupt-driven wrapper for a single TSOP pin.
//   IRBeaconTracker  – manages two IRDetector channels; heading correction
//                      and 2-D localization.
//   IRSweep          – optional active sweep; reports unknown-target bearings.
//
//   IRSweep is entirely opt-in: simply don't construct or call it if you only
//   want heading reference from beacons.
//
// ─── Typical loop() integration ───────────────────────────────────────────────
//
//   IRBeaconTracker beacons;
//   IRSweep         sweep;    // optional
//
//   setup():
//     beacons.init();
//     beacons.calibrate();             // arms auto-calibration from first spin
//     sweep.init();                    // optional
//     sweep.enable();                  // optional
//
//   loop():
//     accelerometers.refresh();
//     mag.update(currentTime);
//     estimator.update(currentTime);
//
//     beacons.update(currentTime, estimator.getAngle(), estimator.getOmega());
//     if (beacons.hasHeadingFix()) {
//         estimator.correctAngle(IR_SNAP_GAIN * beacons.getHeadingError());
//         beacons.clearHeadingFix();
//     }
//
//     sweep.setBeaconAPhase(beacons.getLastAngleA());  // keep filter in sync
//     sweep.setBeaconBPhase(beacons.getLastAngleB());
//     sweep.update(currentTime, estimator.getAngle(), estimator.getOmega());
//     // sweep.getHitCount() / sweep.getHitAngle(i) → opponent candidates
//
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

// ─── Pin defaults ─────────────────────────────────────────────────────────────
// Override any of these in Config.h by defining the symbol before this header
// is included.  These four pins are all free on the Teensy 4.1 after DShot,
// I2C, CRSF, ESC telemetry, and the Serial4 telemetry link are accounted for.

#ifndef IR_BEACON_A_PIN
#define IR_BEACON_A_PIN   2    // TSOP38438 — 38 kHz beacon receiver
#endif
#ifndef IR_BEACON_B_PIN
#define IR_BEACON_B_PIN   3    // TSOP38440 — 40 kHz beacon receiver
#endif
#ifndef IR_SWEEP_RX_PIN
#define IR_SWEEP_RX_PIN  10    // TSOP38436 — 36 kHz sweep receiver
#endif
#ifndef IR_SWEEP_TX_PIN
#define IR_SWEEP_TX_PIN  11    // IR LED    — 36 kHz sweep emitter (PWM)
#endif

// ─── Tuning ───────────────────────────────────────────────────────────────────

// Fraction of the heading error applied each time a beacon fires.
// 0 = correction disabled.  1.0 = instant snap to beacon phase.
// A value around 0.3 is a good starting point — tune during testing.
// Used externally:  estimator.correctAngle(IR_SNAP_GAIN * beacons.getHeadingError())
#ifndef IR_SNAP_GAIN
#define IR_SNAP_GAIN     0.3f
#endif

// Minimum spin rate (rad/s) before IR heading corrections are applied.
// Below this threshold angle interpolation is too noisy to be useful.
// 10 rad/s ≈ 95 RPM.
static constexpr float IR_MIN_OMEGA = 10.0f;

// Angular error (rad) above which a beacon detection is discarded as a
// false positive (noise spike or reflective surface in the arena).
// π/4 = 45°.
static constexpr float IR_MAX_VALID_ERROR = 0.785f;

// A beacon not seen for this long (µs) is considered stale.
// 500 ms gives hundreds of revolutions of headroom at any realistic spin speed.
static constexpr uint32_t IR_STALE_US = 500000UL;

// Angular window (rad) around each known beacon direction within which a sweep
// hit is attributed to the beacon and excluded from the unknown-target list.
// ≈ 20°.
static constexpr float IR_SWEEP_FILTER_RAD = 0.35f;

// PWM carrier frequency for the sweep emitter pin (must match TSOP receiver).
static constexpr uint32_t IR_SWEEP_FREQ = 36000;

// Maximum number of IRDetector instances the static ISR dispatch table supports.
// Raise this and add a corresponding static ISR wrapper in IR.cpp if you ever
// need a fourth channel.
static constexpr int IRDETECTOR_MAX_INSTANCES = 3;

// Ring-buffer capacity for sweep hit storage between clearHits() calls.
static constexpr int IR_SWEEP_MAX_HITS = 32;

// Minimum angular gap (rad) between consecutive sweep hits before a new hit is
// stored.  Prevents a single physical reflector from producing a cluster of
// hits due to ISR re-triggering.  ≈ 5.7°.
static constexpr float IR_SWEEP_MIN_HIT_SPACING = 0.1f;


// ═══════════════════════════════════════════════════════════════════════════════
// IRDetector
// ═══════════════════════════════════════════════════════════════════════════════
//
// Thin interrupt-driven wrapper for a single TSOP receiver pin.
//
// A CHANGE interrupt fires on both the falling edge (receiver first sees the
// carrier — robot sweeps into alignment with the beacon) and the rising edge
// (carrier leaves the field of view — robot sweeps past).  Both timestamps are
// stored so the caller can compute the mid-pulse time, which corresponds to the
// moment the robot was most directly facing the emitting source.
//
// Up to IRDETECTOR_MAX_INSTANCES instances are supported.  The static ISR
// dispatch table is managed automatically inside init().

class IRDetector {
public:
    // Attach a CHANGE interrupt on `pin` and configure it as INPUT_PULLUP.
    // Must be called once during setup() before any reads.
    void init(uint8_t pin);

    // ── Pulse polling ─────────────────────────────────────────────────────────

    // Returns true when a complete falling-then-rising pulse has been captured.
    // Check this every loop iteration; call clearPulse() after consuming the data
    // so the next pulse can be recorded.
    bool hasPulse() const { return _pulseDone; }

    // micros() timestamp at the falling edge — first moment the carrier was seen.
    uint32_t getFallTime() const { return _fallTime; }

    // micros() timestamp at the rising edge — carrier left the field of view.
    uint32_t getRiseTime() const { return _riseTime; }

    // Midpoint of the pulse:  (_fallTime + _riseTime) / 2.
    // This is the best estimate of the moment the robot was pointing directly at
    // the source, and should be used for angle interpolation when precision matters.
    // Note: reads two separate volatile fields — call only after hasPulse() returns
    // true and before clearPulse().
    uint32_t getMidTime() const { return (_fallTime + _riseTime) >> 1; }

    // True while the pin is currently held low (actively inside a pulse window).
    bool isDetecting() const { return _detecting; }

    // Consume and clear the pending pulse.  Call after reading fall/rise/mid times.
    // Uses a brief noInterrupts() guard to prevent a race between the caller's
    // read and a new ISR firing.
    void clearPulse();

    // ── ISR dispatch — not for external use ───────────────────────────────────
    // Called by the static ISR wrapper registered in init().  Public only because
    // static free functions need access to it; do not call from user code.
    void _handleInterrupt();

private:
    uint8_t _pin = 0;

    // All four fields are written inside the ISR and read in main-loop context.
    // On ARM Cortex-M7 (Teensy 4.x), 32-bit and 8-bit reads/writes are atomic,
    // so no explicit lock is needed for individual field accesses.
    volatile uint32_t _fallTime  = 0;
    volatile uint32_t _riseTime  = 0;
    volatile bool     _detecting = false;  // true while pin is held LOW
    volatile bool     _pulseDone = false;  // true once a complete pulse is ready
};


// ═══════════════════════════════════════════════════════════════════════════════
// IRBeaconTracker
// ═══════════════════════════════════════════════════════════════════════════════
//
// Manages two IRDetector channels (one per frequency-coded beacon), derives the
// robot's heading correction, and optionally triangulates (x, y) position.
//
// ── Heading correction ────────────────────────────────────────────────────────
//
// Each beacon has a configured "phase" — the angle (radians) that the robot's
// estimator should read when it sweeps past that beacon.  On every detection,
// the measured angle (back-interpolated to the pulse midpoint) is compared with
// the expected phase; the signed error is exposed to the caller for application
// to AngleEstimator::correctAngle().
//
// Intentionally decoupled from AngleEstimator: the correction step lives in
// main.cpp, allowing the IR system to be dropped in or out without touching the
// estimator class.
//
// ── Localization ──────────────────────────────────────────────────────────────
//
// When both beacons have been seen recently, a ray-intersection calculation uses
// the two measured bearing angles to triangulate the robot's position in metres.
// Requires setBeaconPositions() to be called with the real-world coordinates of
// each beacon.

class IRBeaconTracker {
public:
    // Attach interrupts on both TSOP receiver pins.  Call during setup().
    void init(uint8_t beaconAPin = IR_BEACON_A_PIN,
              uint8_t beaconBPin = IR_BEACON_B_PIN);

    // ── Main update ───────────────────────────────────────────────────────────

    // Call every loop iteration, AFTER estimator.update() so that currentAngle
    // and omega reflect the freshest integration step.
    //
    // currentAngle : estimator.getAngle()  [radians, 0 – 2π]
    // omega        : estimator.getOmega()  [rad/s]
    //
    // Internally:
    //   • Checks each IRDetector for a pending pulse.
    //   • Back-interpolates the angle to the pulse midpoint.
    //   • During calibration mode: records the angle as the beacon's phase.
    //   • Otherwise: computes heading error and sets hasHeadingFix() = true.
    //   • If beacon positions are configured: attempts a localization update.
    void update(uint32_t t_us, float currentAngle, float omega);

    // ── Heading correction ────────────────────────────────────────────────────

    // True when a fresh heading correction is waiting to be consumed.
    bool hasHeadingFix() const { return _hasHeadingFix; }

    // Signed angular error (radians): expected_phase − measured_angle, wrapped
    // to [−π, π].  Multiply by IR_SNAP_GAIN before passing to correctAngle().
    // Positive → robot is slightly behind where it should be when it sees the
    // beacon; negative → slightly ahead.
    float getHeadingError() const { return _headingError; }

    // Consume the pending fix.  Call immediately after reading getHeadingError().
    void clearHeadingFix() { _hasHeadingFix = false; }

    // ── Phase configuration ───────────────────────────────────────────────────

    // Set the expected angle (radians) at which each beacon should be detected.
    // For a simple single-beacon heading reference, set Beacon A to 0 — the
    // beacon then defines your absolute "north".
    void setBeaconAPhase(float rad) { _phaseA = rad; }
    void setBeaconBPhase(float rad) { _phaseB = rad; }

    // Arms auto-calibration: the very next detection of each beacon will be
    // recorded as its reference phase rather than generating a correction.
    // Useful for first-time setup — spin the robot once and it learns where
    // each beacon sits in the angle estimate.
    void calibrate();

    // ── Localization ──────────────────────────────────────────────────────────

    // Provide the arena-frame coordinates of both beacons (metres).
    // Must be called before getX() / getY() will return valid data.
    // Origin and axes are arbitrary — just be consistent.
    void setBeaconPositions(float ax, float ay, float bx, float by);

    // True after a successful position fix has been computed.
    bool  hasPosition() const { return _hasPosition; }
    float getX()        const { return _posX; }   // metres, arena frame
    float getY()        const { return _posY; }   // metres, arena frame

    // ── Diagnostics ───────────────────────────────────────────────────────────

    // Most recently measured angle (radians) at which each beacon was seen.
    // Useful for passing to IRSweep::setBeaconAPhase() to keep the sweep filter
    // in sync with the latest heading estimate.
    float    getLastAngleA()   const { return _lastAngleA; }
    float    getLastAngleB()   const { return _lastAngleB; }

    // True once each beacon has been detected at least once since init().
    bool     seenA()           const { return _seenA; }
    bool     seenB()           const { return _seenB; }

    // micros() timestamps of the most recent detection for each beacon.
    // Compare against micros() to check staleness.
    uint32_t lastSeenTimeA()   const { return _lastSeenUsA; }
    uint32_t lastSeenTimeB()   const { return _lastSeenUsB; }

private:
    IRDetector _detA;  // 38 kHz channel — Beacon A
    IRDetector _detB;  // 40 kHz channel — Beacon B

    // Expected detection angles.  Defaults put beacons on opposite sides of the
    // arena; override with setBeaconAPhase() / setBeaconBPhase().
    float _phaseA = 0.0f;
    float _phaseB = PI;

    // When true, the next detection sets the phase instead of correcting it.
    bool _calibrateA = false;
    bool _calibrateB = false;

    // Latest measured detection angles and their timestamps.
    float    _lastAngleA  = 0.0f;
    float    _lastAngleB  = 0.0f;
    uint32_t _lastSeenUsA = 0;
    uint32_t _lastSeenUsB = 0;
    bool     _seenA       = false;
    bool     _seenB       = false;

    // Pending heading correction.
    float _headingError  = 0.0f;
    bool  _hasHeadingFix = false;

    // Beacon world positions (metres).
    float _ax = 0.0f, _ay = 0.0f;
    float _bx = 1.0f, _by = 0.0f;  // default: beacons 1 m apart on x-axis
    bool  _positionsSet = false;

    // Computed robot position.
    float _posX        = 0.0f;
    float _posY        = 0.0f;
    bool  _hasPosition = false;

    // ── Private helpers ───────────────────────────────────────────────────────

    // Given the current angle and omega, compute the angle the robot had at a
    // past timestamp.  Used to place the detection precisely at the pulse midpoint
    // rather than at the current loop iteration time.
    float _backInterpolate(float currentAngle, float omega,
                           uint32_t t_now_us, uint32_t t_past_us) const;

    // Triangulate (x, y) from two measured bearing angles and the configured
    // beacon world positions.  Returns false if the geometry is degenerate
    // (beacons nearly collinear from the robot's current position).
    bool  _triangulate(float phiA, float phiB);

    float _wrapAngle(float a) const;  // wrap to [0, 2π]
    float _wrapError(float e) const;  // wrap to [−π, π]
};


// ═══════════════════════════════════════════════════════════════════════════════
// IRSweep
// ═══════════════════════════════════════════════════════════════════════════════
//
// Optional active-IR sweep channel for passive opponent / obstacle detection.
//
// The robot continuously emits 36 kHz IR from IR_SWEEP_TX_PIN via hardware PWM
// (no CPU overhead once started).  As it spins, the beam bounces off nearby
// objects; the TSOP38436 on IR_SWEEP_RX_PIN fires each time the robot faces a
// reflector.  Known beacon directions are filtered out; remaining hits are
// stored as unknown-target (opponent / obstacle) bearing candidates.
//
// Usage:
//   Only construct and call this class if you actually want sweep detection.
//   It is completely independent of IRBeaconTracker — either can be used alone.

class IRSweep {
public:
    // Initialise the emitter PWM pin and attach the receiver interrupt.
    // Does NOT start emitting — call enable() when ready.
    void init(uint8_t emitterPin  = IR_SWEEP_TX_PIN,
              uint8_t receiverPin = IR_SWEEP_RX_PIN);

    // ── Emitter control ───────────────────────────────────────────────────────

    // Start the 36 kHz carrier on the emitter pin.  Runs entirely in the
    // FlexPWM hardware after this call — zero ongoing CPU cost.
    void enable();

    // Stop the carrier.
    void disable();

    bool isEnabled() const { return _enabled; }

    // ── Main update ───────────────────────────────────────────────────────────

    // Call every loop iteration, ideally after beacons.update() so that the
    // beacon phases passed to setBeaconAPhase() / setBeaconBPhase() are current.
    //
    // For each pending receiver pulse: back-interpolates the hit angle, filters
    // out known beacon directions, rejects duplicates from ISR jitter, and
    // stores the remaining hit angles.
    //
    // Call clearHits() at the start of each revolution (or on a fixed timer) to
    // reset the hit list.  Hits accumulate until cleared.
    void update(uint32_t t_us, float currentAngle, float omega);

    // ── Hit data ──────────────────────────────────────────────────────────────

    // Number of unknown-target hits since the last clearHits().
    int   getHitCount()          const { return _hitCount; }

    // Arena heading (radians, 0 – 2π) of the i-th hit.
    // Returns 0 for out-of-range index.
    float getHitAngle(int index) const;

    // Clear all stored hits — call at the start of each revolution.
    void  clearHits();

    // ── Beacon filter ─────────────────────────────────────────────────────────

    // Provide the current beacon phases so they can be excluded from hit reports.
    // Typically called with beacons.getLastAngleA() / getLastAngleB() each loop.
    // If never called, ALL reflections are reported (including beacons).
    void setBeaconAPhase(float rad) { _phaseA = rad; _filterBeacons = true; }
    void setBeaconBPhase(float rad) { _phaseB = rad; _filterBeacons = true; }

private:
    IRDetector _receiver;
    uint8_t    _emitterPin  = 0;
    bool       _enabled     = false;

    float _phaseA        = 0.0f;
    float _phaseB        = PI;
    bool  _filterBeacons = false;

    float _hitAngles[IR_SWEEP_MAX_HITS] = {};
    int   _hitCount = 0;

    // Returns true if `angle` is within IR_SWEEP_FILTER_RAD of a known beacon.
    bool  _isKnownBeacon(float angle) const;

    float _wrapAngle(float a) const;  // wrap to [0, 2π]
};

#endif // MELTYBRAIN_IR_H
