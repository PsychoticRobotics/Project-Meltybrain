// IRTest.cpp — IR sensor calibration and diagnostics
//
// Set USE_IR_TEST to 1 below to activate this file.  When active it provides
// its own setup() and loop(), so make sure the normal main.cpp build is not
// included at the same time (easiest: comment out #include lines or use a
// separate PlatformIO environment).
//
// What it does:
//   1. Prints a line to Serial whenever a TSOP fires, showing the current
//      angle estimate and spin rate.
//   2. Runs auto-calibration on the first spin: the first detection of each
//      beacon is recorded as its reference phase.
//   3. Every 5 seconds prints a summary: average measured phase per beacon,
//      position estimate (if beacon positions are configured), and spin stats.
//   4. Logs any sweep hits (unknown reflectors) as they arrive.
//
// How to use:
//   1. Wire up your TSOP receivers and (optionally) the sweep emitter/receiver.
//   2. Set USE_IR_TEST 1 below and rebuild.
//   3. Open the Serial Monitor at 115200 baud.
//   4. Spin the robot up with the beacons placed at their intended positions.
//   5. Note the "Beacon X phase calibrated" angles printed on the first pass.
//      These are your setBeaconAPhase() / setBeaconBPhase() values for the
//      main build — hard-code them in setup() once they are stable.
//   6. Set USE_IR_TEST 0 and rebuild for normal operation.

#define USE_IR_TEST 0   // ← set to 1 to enable, 0 for normal build

#if USE_IR_TEST

#include <Arduino.h>
#include <Wire.h>
#include "IR.h"
#include "Accelerometer.h"
#include "Magnetometer.h"
#include "AngleEstimator.h"

// ─── Sensor objects ───────────────────────────────────────────────────────────
AccelerometerManager accelerometers;
MagnetometerTracker  mag;
AngleEstimator       estimator(accelerometers, mag);

// ─── IR objects ───────────────────────────────────────────────────────────────
IRBeaconTracker beacons;
IRSweep         sweep;

// ─── Calibration accumulators ─────────────────────────────────────────────────
// We average the measured detection angles over several revolutions to get a
// stable phase reference, rather than trusting a single sample.
static float    _sumAngleA = 0.0f, _sumAngleB = 0.0f;
static int      _countA    = 0,    _countB    = 0;
static uint32_t _lastSampleUsA = 0, _lastSampleUsB = 0;

// ─── Timing ───────────────────────────────────────────────────────────────────
static uint32_t _lastSummaryUs = 0;
static const uint32_t SUMMARY_INTERVAL_US = 5000000UL;  // print summary every 5 s

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("============================================");
    Serial.println("  Meltybrain IR — calibration / diagnostics");
    Serial.println("============================================");
    Serial.printf("  Beacon A pin : %d  (38 kHz TSOP)\n", IR_BEACON_A_PIN);
    Serial.printf("  Beacon B pin : %d  (40 kHz TSOP)\n", IR_BEACON_B_PIN);
    Serial.printf("  Sweep RX pin : %d  (36 kHz TSOP)\n", IR_SWEEP_RX_PIN);
    Serial.printf("  Sweep TX pin : %d  (36 kHz LED)\n",  IR_SWEEP_TX_PIN);
    Serial.println("--------------------------------------------");

    // ── Sensors ───────────────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(400000);
    accelerometers.init(0x18);
    if (!mag.init()) {
        Serial.println("[mag] WARNING: magnetometer not found — accel only.");
    }

    // ── IR ────────────────────────────────────────────────────────────────────
    beacons.init(IR_BEACON_A_PIN, IR_BEACON_B_PIN);

    // Arm auto-calibration: the first pass of each beacon sets its phase.
    beacons.calibrate();

    // Optional: set known beacon world positions to enable localization.
    // Units are metres; origin and axes are yours to choose.
    // beacons.setBeaconPositions(0.0f, 1.2f, 1.2f, 0.0f);  // example

    sweep.init(IR_SWEEP_TX_PIN, IR_SWEEP_RX_PIN);
    sweep.enable();

    Serial.println("[IR]  Calibration armed — spin the robot past each beacon.");
    Serial.println("[IR]  Waiting for detections...");
    Serial.println("--------------------------------------------");
}

void loop() {
    uint32_t t = micros();

    // ── Sensor update ─────────────────────────────────────────────────────────
    accelerometers.refresh();
    mag.update(t);
    estimator.update(t);

    float angle = estimator.getAngle();
    float omega = estimator.getOmega();
    float rpm   = omega * (60.0f / (2.0f * PI));

    // ── IR update ─────────────────────────────────────────────────────────────
    beacons.update(t, angle, omega);

    // Keep the sweep beacon filter in sync with the latest beacon angles so
    // confirmed beacon positions are excluded from unknown-target hits.
    if (beacons.seenA()) sweep.setBeaconAPhase(beacons.getLastAngleA());
    if (beacons.seenB()) sweep.setBeaconBPhase(beacons.getLastAngleB());
    sweep.update(t, angle, omega);

    // ── Apply any heading correction (for estimator accuracy during the test) ──
    if (beacons.hasHeadingFix()) {
        float err = beacons.getHeadingError();
        estimator.correctAngle(IR_SNAP_GAIN * err);

        Serial.printf("[fix]  err=% .4f rad  angle=%.4f rad  RPM=%6.1f\n",
                      err, angle, rpm);
        beacons.clearHeadingFix();
    }

    // ── Accumulate samples for summary ────────────────────────────────────────
    // Record the latest angle each time a beacon is freshly seen (deduplicated
    // by comparing the last-seen timestamp against the previous sample time).
    if (beacons.seenA() && beacons.lastSeenTimeA() != _lastSampleUsA) {
        _sumAngleA += beacons.getLastAngleA();
        _countA++;
        _lastSampleUsA = beacons.lastSeenTimeA();
    }
    if (beacons.seenB() && beacons.lastSeenTimeB() != _lastSampleUsB) {
        _sumAngleB += beacons.getLastAngleB();
        _countB++;
        _lastSampleUsB = beacons.lastSeenTimeB();
    }

    // ── Log sweep hits ─────────────────────────────────────────────────────────
    if (sweep.getHitCount() > 0) {
        Serial.printf("[sweep]  %d hit(s): ", sweep.getHitCount());
        for (int i = 0; i < sweep.getHitCount(); i++) {
            Serial.printf("%.1f° ", sweep.getHitAngle(i) * (180.0f / PI));
        }
        Serial.println();
        sweep.clearHits();
    }

    // ── Periodic summary ──────────────────────────────────────────────────────
    if ((t - _lastSummaryUs) >= SUMMARY_INTERVAL_US) {
        _lastSummaryUs = t;

        Serial.println("────────────────────────────────────────────");
        Serial.printf("  RPM: %.0f    omega: %.2f rad/s\n", rpm, omega);

        if (_countA > 0) {
            float avgA = _sumAngleA / _countA;
            Serial.printf("  Beacon A  avg=%.4f rad (%.1f°)  n=%d\n",
                          avgA, avgA * (180.0f / PI), _countA);
            Serial.println("  → setBeaconAPhase(" + String(avgA, 4) + "f)");
        } else {
            Serial.println("  Beacon A: not detected yet.");
        }

        if (_countB > 0) {
            float avgB = _sumAngleB / _countB;
            Serial.printf("  Beacon B  avg=%.4f rad (%.1f°)  n=%d\n",
                          avgB, avgB * (180.0f / PI), _countB);
            Serial.println("  → setBeaconBPhase(" + String(avgB, 4) + "f)");
        } else {
            Serial.println("  Beacon B: not detected yet.");
        }

        if (beacons.hasPosition()) {
            Serial.printf("  Position: (%.3f, %.3f) m\n",
                          beacons.getX(), beacons.getY());
        } else {
            Serial.println("  Position: not computed");
            Serial.println("    (call beacons.setBeaconPositions() to enable)");
        }

        Serial.println("────────────────────────────────────────────");
    }
}

#endif // USE_IR_TEST
