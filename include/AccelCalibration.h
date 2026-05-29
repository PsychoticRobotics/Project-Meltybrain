#pragma once

#include <Arduino.h>

// ─── Configuration ────────────────────────────────────────────────────────────

static constexpr uint8_t  ACCEL_CAL_POINTS       = 8;     // max entries per sensor table
static constexpr uint16_t ACCEL_CAL_ZERO_SAMPLES  = 200;  // readings averaged for zero-G
static constexpr uint16_t ACCEL_CAL_EEPROM_BASE   = 0;    // first EEPROM byte we use

// Increment applied to the working factor per stick-input tick in calibration mode.
// 0.005 = 0.5 % per tap; 100 taps covers the ±50 % range that handles real-world
// sensor variation.
static constexpr float    ACCEL_CAL_FACTOR_STEP   = 0.005f;

// EEPROM validity sentinels — change either byte to force a clean reset after
// a firmware update that alters the EEPROM layout.
static constexpr uint8_t  ACCEL_CAL_SENTINEL_A    = 0xAC;
static constexpr uint8_t  ACCEL_CAL_SENTINEL_B    = 0xC1;

// ─── Data structures ──────────────────────────────────────────────────────────

/**
 * One entry in the piecewise-linear correction table.
 *
 * The correction is applied as:
 *   corrected_G = (raw_G − zeroOffset) × (1 + interpolated_factor)
 *
 * A factor of 0 means no correction.  A factor of +0.05 scales the reading up
 * by 5 %.  The table is kept sorted ascending by .g so interpolation is O(n).
 */
struct CalPoint {
    float g;       // G magnitude at which this entry was captured (always ≥ 0)
    float factor;  // fractional gain correction: 0 = identity, ±0.1 = ±10 %
};

/**
 * Calibration data for one accelerometer's centripetal axis.
 * Two of these are stored in EEPROM (one per physical sensor).
 *
 * sizeof(AxisCal) on Teensy 4.1 (ARM, 4-byte float alignment):
 *   float zeroOffset    =  4 bytes
 *   CalPoint table[8]   = 64 bytes  (8 × 2 floats)
 *   uint8_t len         =  1 byte
 *   uint8_t evictPos    =  1 byte
 *   2 bytes padding to next 4-byte boundary
 *   Total               = 72 bytes
 */
struct AxisCal {
    float    zeroOffset;                  // DC bias at rest, subtracted before any scaling
    CalPoint table[ACCEL_CAL_POINTS];     // sorted ascending by .g
    uint8_t  len;                         // active entries (0 … ACCEL_CAL_POINTS)
    uint8_t  evictPos;                    // ring-buffer pointer for when the table is full
};

// ─── AccelCalibrationManager ──────────────────────────────────────────────────

/**
 * Runtime calibration for both accelerometers' centripetal ("separation") axes.
 *
 * WHAT IT CORRECTS
 * ─────────────────
 * 1. Zero-G DC offset — the LIS331 at ±400 G range can have up to ±2.5 G of
 *    zero-offset error.  Captured once at rest and subtracted on every read.
 *
 * 2. Range-dependent gain nonlinearity — the sensor's sensitivity (G/LSB) is
 *    nominally flat but drifts slightly across the ±400 G range.  Up to
 *    ACCEL_CAL_POINTS operating points are captured during spinning; the manager
 *    linearly interpolates between them at runtime.
 *
 * WHAT IT DOESN'T CORRECT
 * ─────────────────────────
 * Cross-axis contamination and physical mounting misalignment are handled
 * separately by Accelerometer::setAdjustment() (static, compile-time).
 *
 * AXIS NOTE
 * ──────────
 * This manager corrects only one axis per sensor — the "separation axis" whose
 * readings feed the differential ω formula in AngleEstimator.  By default that
 * axis is the raw sensor y (.y() from Accelerometer::fetch()).  If your physical
 * mounting maps the separation axis to a different body axis, update the two
 * .y() references in AccelerometerManager::refresh() and in
 * AccelerometerManager::captureZeroG().
 *
 * HOW CALIBRATION IS APPLIED
 * ───────────────────────────
 *   corrected = (raw − zeroOffset) × (1 + interpolate(|raw − zeroOffset|))
 *
 * EEPROM USAGE
 * ─────────────
 * Starts at ACCEL_CAL_EEPROM_BASE.  Total footprint:
 *   2 sentinel bytes + 2 × sizeof(AxisCal) = 2 + 144 = 146 bytes.
 * Teensy 4.1 provides 4284 bytes of emulated EEPROM — plenty of headroom.
 * All multi-byte fields are written with EEPROM.put() (unlike PotatoMelt which
 * uses the single-byte EEPROM.write() on floats, silently corrupting them).
 *
 * CALIBRATION PROCEDURE (summary — details in main.cpp handleCalibration())
 * ─────────────────────────────────────────────────────────────────────────
 * Step 1  Zero-G capture — robot still, trigger captureZeroG():
 *           Averages ACCEL_CAL_ZERO_SAMPLES reads per sensor, stores mean as
 *           zeroOffset.  Compensates for sensor DC bias and gravity component
 *           along the separation axis.
 *
 * Step 2  Per-RPM gain calibration — robot spinning at a stable RPM:
 *           a. Watch Serial: if "drift °/s" ≠ 0, the computed ω is wrong.
 *           b. Tap sticks to nudge workingFactor up/down until drift ≈ 0.
 *           c. Trigger commitPoint() — records the current G reading and
 *              factor for both sensors simultaneously.
 *           d. Repeat at up to ACCEL_CAL_POINTS different RPMs covering the
 *              full operating range.
 *
 * Step 3  Exit calibration mode — triggers save() to persist to EEPROM.
 */
class AccelCalibrationManager {
public:
    // ── Persistence ──────────────────────────────────────────────────────────

    /**
     * Load calibration from EEPROM.  Call once from setup().
     * If EEPROM is blank or the sentinel bytes don't match (firmware change),
     * silently resets to neutral defaults so the robot still runs.
     */
    void load();

    /**
     * Write current calibration to EEPROM.  Call when exiting calibration mode.
     * EEPROM.put() is used throughout — multi-byte safe, no PotatoMelt float bug.
     */
    void save();

    /**
     * Clear all calibration data and return to neutral defaults.
     * Does NOT write to EEPROM — call save() afterward if you want a clean wipe.
     */
    void reset();

    // ── Runtime correction ────────────────────────────────────────────────────

    /**
     * Apply the full correction to one raw G reading.
     * sensorIdx: 0 = accel1 ("top" sensor), 1 = accel2 ("bottom" sensor).
     * Returns: (raw − zeroOffset) × (1 + piecewise_interpolated_factor)
     * With no calibration data loaded this is simply (raw − 0) × 1 = raw.
     */
    float apply(uint8_t sensorIdx, float rawG) const;

    // ── Calibration session API ───────────────────────────────────────────────

    /**
     * Store the zero-G offset for one sensor.
     * Pass in the mean of ACCEL_CAL_ZERO_SAMPLES raw readings collected at rest
     * (AccelerometerManager::captureZeroG() does the sampling for you).
     */
    void captureZero(uint8_t sensorIdx, float mean);

    /**
     * The correction factor currently being tuned.
     * Adjusted interactively in calibration mode; committed with commitPoint().
     */
    float workingFactor() const          { return _workingFactor; }
    void  setWorkingFactor(float f)      { _workingFactor = f; }
    void  adjustFactor(float delta)      { _workingFactor += delta; }
    void  resetWorkingFactor()           { _workingFactor = 0.0f; }

    /**
     * Commit the current workingFactor to both sensors' tables.
     * gSensor0 / gSensor1: the raw G reading on the separation axis at the
     * moment of commit for each sensor (AccelerometerManager::fetchXYZ1/2().y()).
     * A sorted insert is performed; uses ring-buffer eviction if the table is full.
     * Both sensors receive the same factor because they measure the same rotation.
     */
    void commitPoint(float gSensor0, float gSensor1);

    /** Clear the correction table for one sensor (leaves zeroOffset intact). */
    void clearTable(uint8_t sensorIdx);

    /** Clear both sensors' tables. */
    void clearAllTables();

    // ── Diagnostics ──────────────────────────────────────────────────────────

    /** Print the full table contents for both sensors to Serial. */
    void printTable() const;

    const AxisCal& calData(uint8_t idx) const { return _cal[idx & 1]; }
    bool isDirty() const { return _dirty; }

private:
    AxisCal _cal[2];
    float   _workingFactor = 0.0f;
    bool    _dirty         = false;

    /** Linear interpolation between the two surrounding table entries. */
    float interpolate(const AxisCal& c, float absG) const;

    /**
     * Insert (g, factor) into sensor idx's table in sorted order.
     * If the table is full, the entry at evictPos is overwritten (ring buffer),
     * then the table is re-sorted with an insertion sort (max 8 elements).
     */
    void sortedInsert(uint8_t sensorIdx, float g, float factor);
};
