#ifndef MELTYBRAIN_AUTONOMY_H
#define MELTYBRAIN_AUTONOMY_H

// ─── Autonomy ─────────────────────────────────────────────────────────────────
//
// Handles all four drive modes and the autonomous state machine.
//
// Typical loop() integration:
//
//   DriveCommand cmd = autonomy.update(
//       channels, estimator.getOmega(),
//       pos_x, pos_y, pos_valid,
//       opp_bearing, opp_valid,
//       wall_dist, wall_bearing
//   );
//   if (cmd.isTank) motors.on(cmd.left, cmd.right);
//   else            robot.move(cmd.ch1_us, cmd.ch2_us, cmd.ch3_us);
//
// Mode is selected by an RC switch on MODE_SELECT_CH (see Config.h).

#include <stdint.h>
#include <Arduino.h>
#include "Config.h"

// ─── Mode and state enums ─────────────────────────────────────────────────────

enum class RobotMode : uint8_t {
    TANK     = 0,   // non-spinning differential drive
    MELTY    = 1,   // RC-controlled melty-brain translation
    ASSISTED = 2,   // melty + wall avoidance + opponent seek blended over RC
    AUTO     = 3,   // fully autonomous state machine
};

enum class AutoState : uint8_t {
    IDLE    = 0,   // motors off
    SPIN_UP = 1,   // spinning to operating speed; no translation yet
    SEEK    = 2,   // at speed, searching for / approaching opponent
    ATTACK  = 3,   // opponent confirmed — commit
    EVADE   = 4,   // near wall or recovering from impact — get clear
};

// ─── Drive command ────────────────────────────────────────────────────────────
// Produced by Autonomy::update().
// isTank == true  → call motors.on(left, right)
// isTank == false → call robot.move(ch1_us, ch2_us, ch3_us)

struct DriveCommand {
    bool  isTank;
    // TANK fields
    float left;          // left  motor throttle, [-1, 1]
    float right;         // right motor throttle, [-1, 1]
    // MELTY fields (raw µs values)
    float ch1_us;        // lateral channel   (range 994–2014)
    float ch2_us;        // forward channel   (range 990–2010)
    float ch3_us;        // spin throttle     (range 1000–2014)
};

// ─── Autonomy ─────────────────────────────────────────────────────────────────

class Autonomy {
public:
    // Main update — call every loop after sensors/estimators have run.
    DriveCommand update(
        const uint16_t* rc,            // raw RC channels (µs)
        float omega,                    // current spin rate (rad/s)
        float pos_x,   float pos_y,    // arena position (m from centre)
        bool  pos_valid,
        float opp_bearing,             // bearing to nearest opponent (rad)
        bool  opp_valid,
        float wall_dist,               // distance to nearest wall (m)
        float wall_bearing             // bearing to nearest wall (rad)
    );

    RobotMode getMode()  const { return _mode; }
    AutoState getState() const { return _state; }
    void      printState()    const;

private:
    // Per-mode handlers
    DriveCommand _tank    (const uint16_t* rc);
    DriveCommand _melty   (const uint16_t* rc);
    DriveCommand _assisted(const uint16_t* rc, float omega,
                           float opp_bearing, bool opp_valid,
                           float wall_dist,   float wall_bearing);
    DriveCommand _auto    (float omega, float dt,
                           float opp_bearing, bool opp_valid,
                           float wall_dist,   float wall_bearing);

    void _changeState(AutoState s);

    // Build a melty DriveCommand from a desired arena-frame translation.
    // bearing: direction to move toward (rad, arena frame)
    // mag:     translation magnitude [0, 1]
    // throttle: spin motor throttle  [0, 1]
    static DriveCommand _steer(float bearing, float mag, float throttle);

    // RC ch1/ch2/ch3 passed through to a melty DriveCommand unchanged.
    static DriveCommand _rcPassthrough(const uint16_t* rc);

    // Map a µs value from (in_lo, in_hi) to [-1, 1], clamped.
    static float _mapUs(float us, float in_lo, float in_hi);

    // State
    RobotMode _mode         = RobotMode::MELTY;
    AutoState _state        = AutoState::IDLE;
    uint32_t  _stateMs      = 0;      // millis() when current state was entered
    uint32_t  _lastUpdateMs = 0;      // for dt computation inside update()
    float     _searchDir    = 0.0f;   // slowly-rotating search bearing (rad)
    uint32_t  _lastOppMs    = 0;      // millis() of last opponent sighting
};

#endif // MELTYBRAIN_AUTONOMY_H
