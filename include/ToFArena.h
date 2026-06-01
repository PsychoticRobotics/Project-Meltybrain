#ifndef MELTYBRAIN_TOF_ARENA_H
#define MELTYBRAIN_TOF_ARENA_H

// ─── ToFArenaMapper ───────────────────────────────────────────────────────────
//
// Manages a TMF8801 time-of-flight sensor and builds a 360° polar distance map
// of the arena. The sensor free-runs at maximum rate; each reading is tagged
// with the robot's current angle (from AngleEstimator) and accumulated into a
// 72-bin polar map (5° per bin) using an EMA per bin.
//
// Readings that are significantly closer than the established wall distance in
// that direction are flagged as potential opponent returns rather than being
// folded into the wall estimate.
//
// Pipeline delay compensation: the TMF8801 takes ~0.5 ms from trigger to
// result. At high spin rates the robot has rotated during that window. Each
// reading is back-tagged to the angle at the midpoint of the measurement using
// the current omega, so the polar map stays aligned.
//
// Wiring (I2C, shared bus with accelerometers):
//   SDA  →  Teensy pin 18  (shared)
//   SCL  →  Teensy pin 19  (shared)
//   INT  →  TOF_INT_PIN    (data-ready interrupt, avoids polling)
//   EN   →  3.3 V          (or a GPIO if you want software power-cycling)
//
// Library: install "SparkFun TMF882X Arduino Library" via PlatformIO
//   lib_deps = sparkfun/SparkFun TMF882X Arduino Library
//
// NOTE: The TMF882X library targets the multizone TMF882X family. The TMF8801
// (single-zone) uses a similar but distinct I2C protocol. If you use the
// TMF8801 specifically, swap in the ams-OSRAM Arduino_ams_TMF8x0x driver from:
//   https://github.com/ams-OSRAM/arduino-tof
// and update the sensor section of ToFArena.cpp accordingly. The polar map
// logic is library-independent and does not need to change.

#include <Arduino.h>
#include <math.h>

// ─── Configuration ────────────────────────────────────────────────────────────

static constexpr int     TOF_NUM_BINS       = 72;      // 5° per bin — 360 / 72
static constexpr float   TOF_BIN_RAD        = (2.0f * (float)M_PI) / TOF_NUM_BINS;
static constexpr float   TOF_MAX_RANGE_M    = 2.5f;    // TMF8801 rated max (m)
static constexpr float   TOF_MIN_RANGE_M    = 0.05f;   // ignore very close returns (m)
static constexpr uint8_t TOF_MIN_CONF       = 50;      // 0–255 — reject low-confidence reads
static constexpr float   TOF_EMA_ALPHA      = 0.15f;   // bin smoothing weight (lower = slower)
static constexpr float   TOF_OPP_THRESHOLD  = 0.35f;   // m — reading this much closer than the
                                                        //     established wall → possible opponent
static constexpr uint8_t TOF_I2C_ADDR      = 0x41;    // TMF8801 default I2C address
static constexpr float   TOF_PIPELINE_MS    = 0.5f;    // measurement pipeline delay (ms)
                                                        // tune against known geometry if needed

#ifndef TOF_INT_PIN
#define TOF_INT_PIN  2   // data-ready interrupt pin — change to suit your wiring
#endif

// ─── ToFArenaMapper ───────────────────────────────────────────────────────────

class ToFArenaMapper {
public:
    // Call from setup() after Wire.begin().
    // Returns false if sensor not found on I2C bus.
    bool init();

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
    // Fills dist_m and confidence; returns true if a fresh reading was obtained.
    bool _readSensor(float* dist_m, uint8_t* confidence);
};

#endif // MELTYBRAIN_TOF_ARENA_H
