#ifndef MELTYBRAIN_IRARENA_H
#define MELTYBRAIN_IRARENA_H

// ─── IR Arena Tracker ────────────────────────────────────────────────────────
//
// Beacon-free localization for a spinning meltybrain robot in a square arena.
//
// Uses the same hardware as IRSweep (36 kHz emitter + TSOP receiver), but
// instead of relying on external beacons for heading correction and position,
// it tracks reflections off the arena walls themselves.
//
// As the robot spins, the IR emitter sweeps 360°.  Each revolution the TSOP
// detects reflections from the wooden kickplates at the base of the arena
// walls (diffuse IR reflectors at robot height).  For each reflection the
// tracker records:
//
//   bearing       – arena-frame angle at the pulse midpoint
//   angular width – pulse duration × omega → how wide the wall appears
//
// From these measurements, the tracker derives:
//
//   1. Heading correction  – wall bearings should fall on a 90° grid;
//                            deviation = heading drift.
//   2. (x, y) position     – angular-width ratio of opposite wall pairs
//                            encodes distance: closer wall → wider return.
//   3. Wall proximity      – nearest wall distance and bearing.
//   4. Opponent detection   – reflections that don't match any tracked wall
//                            are reported as unknown targets.
//
// ─── How it works ─────────────────────────────────────────────────────────────
//
// Hits are accumulated over one full revolution (detected by the estimator's
// angle wrapping past 2π).  At the revolution boundary:
//
//   1. New hits are matched to existing tracked wall slots by bearing.
//   2. Matched walls update via EMA; persistence counter increments.
//   3. After ARENA_WALL_MIN_PERSISTENCE revolutions a slot is classified
//      as a confirmed wall.
//   4. Unmatched hits are reported as opponent candidates.
//   5. Slots not seen for ARENA_WALL_DECAY_REVS are removed.
//   6. With ≥ 2 confirmed walls forming an opposite pair, position is
//      estimated from the angular-width ratio.
//   7. Heading correction is derived from the wall bearing pattern's
//      deviation from the calibrated 90° grid.
//
// ─── Hardware ─────────────────────────────────────────────────────────────────
//
//   Same pins as IRSweep — do NOT initialise both IRSweep and IRArenaTracker.
//
//   TSOP38436  → IR_SWEEP_RX_PIN   36 kHz receiver
//   IR LED     ← IR_SWEEP_TX_PIN   36 kHz emitter (PWM)
//
//   No external beacons needed.  Pins 2 and 3 are free in this mode.
//
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "IR.h"   // IRDetector, pin defaults, IR_SWEEP_FREQ, etc.

// ─── Tuning constants ─────────────────────────────────────────────────────────

// Maximum hits buffered per revolution.  4 walls + a few opponents.
static constexpr int ARENA_HITS_PER_REV = 16;

// Maximum tracked wall/candidate slots.  A square arena needs 4; the extra
// slots absorb transient objects while they're being classified.
static constexpr int ARENA_MAX_WALLS = 6;

// A new hit within this angle (rad) of an existing slot is matched to it.
// 0.26 rad ≈ 15°.
static constexpr float ARENA_WALL_MATCH_RAD = 0.26f;

// If a tracked slot's bearing shifts by more than this (rad) between
// consecutive revolutions, its persistence counter resets — it's moving,
// not a wall.  0.05 rad ≈ 3°.
static constexpr float ARENA_WALL_MAX_DRIFT = 0.05f;

// Revolutions of consistent detection before a slot is classified as a wall.
static constexpr uint16_t ARENA_WALL_MIN_PERSISTENCE = 5;

// Revolutions without a match before a slot is removed.
static constexpr uint8_t ARENA_WALL_DECAY_REVS = 3;

// Minimum spin rate for useful wall measurements.
// 15 rad/s ≈ 143 RPM — below this the angle resolution is too coarse.
static constexpr float ARENA_MIN_OMEGA = 15.0f;

// EMA smoothing factor for wall bearing and width updates (0–1).
// Higher = more responsive, lower = more stable.
static constexpr float ARENA_EMA_ALPHA = 0.3f;

// Maximum opponent-hit buffer.
static constexpr int ARENA_MAX_OPPONENT_HITS = 16;

// Heading correction gain — same role as IR_SNAP_GAIN for beacons.
// Applied externally: estimator.correctAngle(ARENA_SNAP_GAIN * error)
#ifndef ARENA_SNAP_GAIN
#define ARENA_SNAP_GAIN  0.3f
#endif


// ─── Data types ──────────────────────────────────────────────────────────────

struct ArenaWall {
    float    bearing;        // smoothed center bearing (radians, 0–2π)
    float    angularWidth;   // smoothed angular extent (radians)
    uint16_t persistence;    // consecutive revolutions seen
    uint8_t  missCount;      // consecutive revolutions NOT seen
    bool     isWall;         // true once persistence ≥ ARENA_WALL_MIN_PERSISTENCE
};


// ═══════════════════════════════════════════════════════════════════════════════
// IRArenaTracker
// ═══════════════════════════════════════════════════════════════════════════════

class IRArenaTracker {
public:

    // ── Setup ─────────────────────────────────────────────────────────────────

    /// Attach receiver interrupt and configure emitter PWM.
    /// Uses the same pins as IRSweep — do NOT initialise both.
    void init(uint8_t emitterPin  = IR_SWEEP_TX_PIN,
              uint8_t receiverPin = IR_SWEEP_RX_PIN);

    /// Set the arena as a square with side length in metres.
    /// Must be called for getX() / getY() / getDistanceInDirection() to work.
    /// Default: 8 ft (2.44 m).
    void setSquareArena(float sideLength_m);

    // ── Emitter control ───────────────────────────────────────────────────────

    void enable();    ///< Start 36 kHz PWM on the emitter LED.
    void disable();   ///< Stop emitter.
    bool isEnabled() const { return _enabled; }

    // ── Main update (call every loop iteration) ───────────────────────────────

    /// After estimator.update() — needs currentAngle and omega.
    void update(uint32_t t_us, float currentAngle, float omega);

    // ── Position output ───────────────────────────────────────────────────────

    bool  hasPosition() const { return _hasPosition; }
    float getX()        const { return _posX; }   ///< metres from arena centre
    float getY()        const { return _posY; }   ///< metres from arena centre

    /// Distance from (posX, posY) to the nearest arena wall along `bearing`.
    /// Useful for movement planning: "how far can I go in this direction?"
    /// Returns 999 if position is unknown.
    float getDistanceInDirection(float bearing) const;

    // ── Wall data ─────────────────────────────────────────────────────────────

    int              getWallCount()            const;   ///< classified walls only
    const ArenaWall& getWall(int index)        const;   ///< index among classified walls
    float            getNearestWallDist()      const;   ///< metres (999 if unknown)
    float            getNearestWallBearing()   const;   ///< radians

    // ── Heading correction ────────────────────────────────────────────────────

    bool  hasHeadingFix()   const { return _hasHeadingFix; }
    float getHeadingError() const { return _headingError; }
    void  clearHeadingFix()       { _hasHeadingFix = false; }

    // ── Opponent hits (non-wall reflections) ──────────────────────────────────

    int   getOpponentHitCount()          const { return _opponentHitCount; }
    float getOpponentHitAngle(int index) const;
    void  clearOpponentHits()                  { _opponentHitCount = 0; }

    // ── Diagnostics ───────────────────────────────────────────────────────────

    void     printWalls()        const;   ///< dump wall table to Serial
    uint32_t getRevolutionCount() const { return _revCount; }

private:
    IRDetector _receiver;
    uint8_t    _emitterPin = 0;
    bool       _enabled    = false;

    // Arena geometry
    float _arenaSize       = 2.44f;   // default 8 ft ≈ 2.44 m
    bool  _arenaConfigured = false;

    // Per-revolution hit buffer
    struct RawHit {
        float bearing;
        float angularWidth;
    };
    RawHit _revHits[ARENA_HITS_PER_REV];
    int    _revHitCount = 0;

    // Revolution tracking
    float    _prevAngle  = 0.0f;
    uint32_t _lastRevUs  = 0;
    uint32_t _revCount   = 0;

    // Tracked wall/candidate slots
    ArenaWall _walls[ARENA_MAX_WALLS];
    int       _slotCount = 0;

    // Position
    float _posX = 0.0f;
    float _posY = 0.0f;
    bool  _hasPosition = false;

    // Heading
    float _arenaRotation           = 0.0f;    // learned arena axis angle (rad)
    bool  _arenaRotationCalibrated = false;
    float _headingError            = 0.0f;
    bool  _hasHeadingFix           = false;

    // Opponent hits
    float _opponentAngles[ARENA_MAX_OPPONENT_HITS];
    int   _opponentHitCount = 0;

    // ── Internal methods ──────────────────────────────────────────────────────
    void  _processRevolution(float omega);
    void  _estimateHeading();
    void  _estimatePosition();
    float _wrapAngle(float a)           const;
    float _angleDist(float a, float b)  const;   // shortest arc [0, π]
};

#endif // MELTYBRAIN_IRARENA_H
