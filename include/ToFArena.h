#ifndef MELTYBRAIN_TOF_ARENA_H
#define MELTYBRAIN_TOF_ARENA_H

// ─── ToFArenaMapper ───────────────────────────────────────────────────────────
//
// Manages a Benewake TFMini-S LiDAR module over UART and builds a 360° polar
// distance map of the arena. The sensor free-runs at up to 1000 Hz; each
// reading is tagged with the robot's current angle (from AngleEstimator) and
// accumulated into a 72-bin polar map (5° per bin) using an EMA per bin.
//
// Readings that are significantly closer than the established wall distance in
// that direction are flagged as potential opponent returns rather than being
// folded into the wall estimate.
//
// Pipeline delay compensation: the TFMini-S takes ~1 ms from pulse to result
// (including the UART transmission of the 9-byte frame at 115200 baud). At high
// spin rates the robot has rotated during that window. Each reading is
// back-tagged to the angle at the midpoint of the measurement using the current
// omega, so the polar map stays aligned.
//
// Wiring (UART, Serial2 on Teensy 4.1):
//   Sensor TX   →  Teensy pin 7   (Serial2 RX)
//   Sensor RX   →  Teensy pin 8   (Serial2 TX — only needed if reconfiguring sensor)
//   5V          →  Teensy 5V (TFMini-S draws ~140 mA, do not power from 3.3V)
//   GND         →  Teensy GND
//
// NOTE: Before using at 1000 Hz, the sensor must be configured to run at that
// rate (default is 100 Hz). At 1000 Hz the 9-byte frames need 90,000 bits/sec
// of UART bandwidth — 115200 baud handles this with ~25% headroom. If you push
// to higher rates or get frame errors, reconfigure both the sensor and
// TFMINI_BAUD below to 230400 or 460800.

#include <Arduino.h>
#include <math.h>

// ─── Configuration ────────────────────────────────────────────────────────────

static constexpr int      TOF_NUM_BINS        = 72;      // 5° per bin — 360 / 72
static constexpr float    TOF_BIN_RAD         = (2.0f * (float)M_PI) / TOF_NUM_BINS;
static constexpr float    TOF_MAX_RANGE_M     = 2.5f;    // ignore returns beyond this (m)
static constexpr float    TOF_MIN_RANGE_M     = 0.10f;   // TFMini-S minimum reliable range
static constexpr uint16_t TOF_MIN_STRENGTH    = 100;     // 0–65535 — reject weak returns
                                                          // per Benewake: <100 is unreliable
static constexpr float    TOF_EMA_ALPHA       = 0.15f;   // bin smoothing weight (lower = slower)
static constexpr float    TOF_OPP_THRESHOLD   = 0.35f;   // m — reading this much closer than the
                                                          //     established wall → possible opponent
static constexpr uint32_t TFMINI_BAUD         = 115200;  // TFMini-S default UART baud
static constexpr float    TOF_PIPELINE_MS     = 1.0f;    // measurement + UART pipeline delay (ms)
                                                          // tune against known geometry if needed

// ─── ToFArenaMapper ───────────────────────────────────────────────────────────

class ToFArenaMapper {
public:
    // Call from setup(). Opens Serial2 at TFMINI_BAUD.
    // Always returns true — there is no init handshake with the TFMini-S, it
    // just starts streaming once powered. Use binValid() / validBinCount() to
    // confirm the sensor is actually producing data after a few revolutions.
    bool init();

    // ── One-time TFMini-S configuration ──────────────────────────────────────
    //
    // TFMini-S ships configured for 100 Hz output. For the arena mapper to be
    // useful at combat spin rates we need 1000 Hz. Call configure1000Hz() ONCE
    // (e.g. by uncommenting a call to it in setup()) with the sensor connected,
    // then re-comment the line — the new frame rate is persisted in the
    // sensor's flash via the Save Settings command.
    //
    // The call blocks for ~200 ms (3 commands × ~50 ms each + flash write).
    // It must be called AFTER init() has opened Serial2.
    //
    // Frame rate command (TFMini-S protocol):
    //   0x5A 0x06 0x03 FR_L FR_H CRC    where FR = rate in little-endian
    //     1000 Hz = 0x03E8  →  0x5A 0x06 0x03 0xE8 0x03 0x48
    // Save-to-flash command:
    //   0x5A 0x04 0x11 0x6F
    //
    // After save, the new rate persists across power cycles. You do NOT need
    // to call configure1000Hz() on every boot.
    void configure1000Hz();

    // Call every loop() after estimator.update().
    //   angle_rad   : current robot heading in [0, 2π) from AngleEstimator
    //   omega_rad_s : current spin rate — used to compensate pipeline delay
    void update(float angle_rad, float omega_rad_s);

    // ── Polar map queries ─────────────────────────────────────────────────────

    // True if a bin has received at least one valid wall reading.
    bool binValid(int bin) const { return _bins[bin].valid; }

    // Smoothed wall distance (m) for a bin index.
    // Returns TOF_MAX_RANGE_M if the bin has no valid readings yet.
    float binDistance(int bin) const;

    // Distance at an arbitrary angle (rad), linearly interpolated between the
    // two surrounding bins. Returns TOF_MAX_RANGE_M if neither bin is valid.
    float distanceAtAngle(float angle_rad) const;

    // True if the most recent reading in this bin was flagged as a potential
    // opponent return (much closer than established wall distance).
    bool binIsOpponent(int bin) const { return _bins[bin].isOpponent; }

    // Bearing (rad) to the closest valid bin.
    float nearestBearing() const;

    // Distance (m) of the closest valid bin.
    float nearestDistance() const;

    // How many bins currently have valid wall readings.
    int validBinCount() const;

    // Convert between angle and bin index.
    static int   angleToBin(float angle_rad);
    static float binToAngle(int bin);   // centre angle of the bin

    // Reset all bins.
    void clear();

    // Dump the full map to Serial (one bin per line).
    void printMap() const;

private:
    struct Bin {
        float   dist       = TOF_MAX_RANGE_M;
        bool    valid      = false;
        bool    isOpponent = false;
    };

    Bin  _bins[TOF_NUM_BINS];
    bool _sensorReady = false;

    // Add one validated distance reading at the given angle.
    void _addReading(float angle_rad, float dist_m);

    // Read one sample from the sensor if one is available.
    // Fills dist_m and strength; returns true if a fresh frame was obtained.
    bool _readSensor(float* dist_m, uint16_t* strength);
};

#endif // MELTYBRAIN_TOF_ARENA_H
