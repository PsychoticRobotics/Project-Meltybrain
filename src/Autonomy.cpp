#include "Autonomy.h"
#include <math.h>

// ─── RC channel µs ranges (must match robot.move() expectations) ──────────────
static constexpr float CH1_LO = 994.0f,  CH1_HI = 2014.0f;   // lateral
static constexpr float CH2_LO = 990.0f,  CH2_HI = 2010.0f;   // forward
static constexpr float CH3_LO = 1000.0f, CH3_HI = 2014.0f;   // throttle

static constexpr float CH1_MID  = (CH1_LO + CH1_HI) * 0.5f;  // 1504
static constexpr float CH2_MID  = (CH2_LO + CH2_HI) * 0.5f;  // 1500
static constexpr float CH1_HALF = (CH1_HI - CH1_LO) * 0.5f;  // 510
static constexpr float CH2_HALF = (CH2_HI - CH2_LO) * 0.5f;  // 510

// AUTO state-machine timings
static constexpr uint32_t OPP_LOST_MS  = 1500;   // ms — leave ATTACK if opponent unseen
static constexpr uint32_t EVADE_MIN_MS =  800;   // ms — minimum time spent in EVADE

// SEEK search pattern
static constexpr float SEARCH_RAD_S = 0.8f;   // rad/s — how fast the search bearing rotates


// ─── Public: update ───────────────────────────────────────────────────────────

DriveCommand Autonomy::update(
    const uint16_t* rc,
    float omega,
    float pos_x, float pos_y, bool pos_valid,
    float opp_bearing, bool opp_valid,
    float wall_dist, float wall_bearing)
{
    // dt for smooth search-pattern rotation.
    uint32_t now = millis();
    float dt = (_lastUpdateMs == 0) ? 0.001f
                                    : (float)(now - _lastUpdateMs) * 0.001f;
    _lastUpdateMs = now;
    dt = fmaxf(0.0f, fminf(0.1f, dt));   // clamp to [0, 100ms] — guard against stalls

    // ── Mode detection from RC switch ─────────────────────────────────────────
    uint16_t  mode_us = rc[MODE_SELECT_CH];
    RobotMode newMode;
    if      (mode_us < MODE_THRESHOLD_TANK) newMode = RobotMode::TANK;
    else if (mode_us < MODE_THRESHOLD_MELTY) newMode = RobotMode::MELTY;
    else if (mode_us < MODE_THRESHOLD_ASST)  newMode = RobotMode::ASSISTED;
    else                                      newMode = RobotMode::AUTO;

    if (newMode != _mode) {
        _mode  = newMode;
        _state = AutoState::IDLE;  // reset state machine whenever mode changes
        Serial.printf("[Autonomy] Mode → %s\n",
            _mode == RobotMode::TANK     ? "TANK"     :
            _mode == RobotMode::MELTY    ? "MELTY"    :
            _mode == RobotMode::ASSISTED ? "ASSISTED" : "AUTO");
    }

    // ── Dispatch ──────────────────────────────────────────────────────────────
    switch (_mode) {
        case RobotMode::TANK:
            return _tank(rc);
        case RobotMode::MELTY:
            return _melty(rc);
        case RobotMode::ASSISTED:
            return _assisted(rc, omega, opp_bearing, opp_valid, wall_dist, wall_bearing);
        case RobotMode::AUTO:
            return _auto(omega, dt, opp_bearing, opp_valid, wall_dist, wall_bearing);
    }
    return _melty(rc);  // unreachable — silences compiler warning
}


// ─── TANK ─────────────────────────────────────────────────────────────────────
// Standard arcade/tank mixer:
//   forward/back (ch2) = both motors at same speed
//   left/right   (ch1) = differential turn
//   left  = fwd + turn
//   right = fwd − turn

DriveCommand Autonomy::_tank(const uint16_t* rc) {
    float turn = _mapUs(rc[0], CH1_LO, CH1_HI);   // lateral,  [-1, 1]
    float fwd  = _mapUs(rc[1], CH2_LO, CH2_HI);   // forward,  [-1, 1]

    DriveCommand cmd = {};
    cmd.isTank = true;
    cmd.left   = fmaxf(-1.0f, fminf(1.0f, fwd + turn));
    cmd.right  = fmaxf(-1.0f, fminf(1.0f, fwd - turn));
    return cmd;
}


// ─── MELTY ────────────────────────────────────────────────────────────────────
// Pure RC pass-through.  robot.move() handles the melty translation timing.

DriveCommand Autonomy::_melty(const uint16_t* rc) {
    return _rcPassthrough(rc);
}


// ─── ASSISTED ────────────────────────────────────────────────────────────────
// RC translation with two autonomy nudges blended in:
//   • Opponent seek: pull toward detected opponent
//   • Wall avoidance: push away from nearest wall when close
// Throttle (ch3) is passed straight through from RC.

DriveCommand Autonomy::_assisted(
    const uint16_t* rc, float omega,
    float opp_bearing, bool opp_valid,
    float wall_dist,   float wall_bearing)
{
    // Decompose RC sticks into a normalised translation vector.
    float rc_x = _mapUs(rc[0], CH1_LO, CH1_HI);   // [-1, 1]
    float rc_y = _mapUs(rc[1], CH2_LO, CH2_HI);   // [-1, 1]

    float auto_x = 0.0f;
    float auto_y = 0.0f;

    // Wall avoidance: push away from wall, strongest when closest.
    if (wall_dist < WALL_AVOID_DIST) {
        float strength = 1.0f - (wall_dist / WALL_AVOID_DIST);   // 0→1 as wall closes in
        float away     = wall_bearing + PI;
        auto_x += WALL_AVOID_GAIN * strength * sinf(away);
        auto_y += WALL_AVOID_GAIN * strength * cosf(away);
    }

    // Opponent seek: nudge in opponent direction (only when fast enough to translate).
    if (opp_valid && omega >= MIN_DRIVE_OMEGA) {
        auto_x += OPP_SEEK_GAIN * sinf(opp_bearing);
        auto_y += OPP_SEEK_GAIN * cosf(opp_bearing);
    }

    // Blend with RC and clamp back to [-1, 1] normalised range.
    float bx = fmaxf(-1.0f, fminf(1.0f, rc_x + auto_x));
    float by = fmaxf(-1.0f, fminf(1.0f, rc_y + auto_y));

    DriveCommand cmd = {};
    cmd.isTank = false;
    cmd.ch1_us = CH1_MID + bx * CH1_HALF;
    cmd.ch2_us = CH2_MID + by * CH2_HALF;
    cmd.ch3_us = (float)rc[2];   // throttle unchanged from pilot
    return cmd;
}


// ─── AUTO state machine ───────────────────────────────────────────────────────
//
// States and transitions:
//
//   IDLE  ──always──►  SPIN_UP
//
//   SPIN_UP  ──ω ≥ SPIN_UP_OMEGA──►  SEEK
//
//   SEEK  ──opp visible──►  ATTACK
//         ──wall close───►  EVADE
//         ──ω too low────►  SPIN_UP
//
//   ATTACK  ──opp lost (1.5 s)──►  SEEK
//            ──wall close──────►  EVADE
//            ──ω too low───────►  SPIN_UP
//
//   EVADE  ──(≥800 ms AND wall safe)──►  SEEK or SPIN_UP
//
// Actions:
//   IDLE    : stop everything
//   SPIN_UP : full throttle, no translation
//   SEEK    : moderate throttle; steer toward opponent or slowly circle to search
//   ATTACK  : full throttle; steer toward opponent bearing
//   EVADE   : moderate throttle; steer directly away from wall

DriveCommand Autonomy::_auto(
    float omega, float dt,
    float opp_bearing, bool opp_valid,
    float wall_dist,   float wall_bearing)
{
    uint32_t now     = millis();
    uint32_t inState = now - _stateMs;

    // ── Track last opponent sighting ──────────────────────────────────────────
    if (opp_valid) _lastOppMs = now;

    // ── State transitions ─────────────────────────────────────────────────────
    switch (_state) {
        case AutoState::IDLE:
            _changeState(AutoState::SPIN_UP);
            break;

        case AutoState::SPIN_UP:
            if (omega >= SPIN_UP_OMEGA)
                _changeState(AutoState::SEEK);
            break;

        case AutoState::SEEK:
            if      (wall_dist < WALL_AVOID_DIST)  _changeState(AutoState::EVADE);
            else if (opp_valid)                     _changeState(AutoState::ATTACK);
            else if (omega < MIN_DRIVE_OMEGA)       _changeState(AutoState::SPIN_UP);
            break;

        case AutoState::ATTACK:
            if      (wall_dist < WALL_AVOID_DIST)             _changeState(AutoState::EVADE);
            else if ((now - _lastOppMs) > OPP_LOST_MS)        _changeState(AutoState::SEEK);
            else if (omega < MIN_DRIVE_OMEGA)                  _changeState(AutoState::SPIN_UP);
            break;

        case AutoState::EVADE:
            if (inState > EVADE_MIN_MS && wall_dist > WALL_SAFE_DIST)
                _changeState(omega >= SPIN_UP_OMEGA ? AutoState::SEEK : AutoState::SPIN_UP);
            break;
    }

    // ── Advance search bearing (for SEEK with no opponent) ────────────────────
    _searchDir = fmodf(_searchDir + SEARCH_RAD_S * dt, 2.0f * PI);

    // ── Actions ───────────────────────────────────────────────────────────────
    switch (_state) {
        case AutoState::IDLE:
            return _steer(0.0f, 0.0f, 0.0f);

        case AutoState::SPIN_UP:
            // Throttle up, no translation until we're at speed.
            return _steer(0.0f, 0.0f, SEEK_THROTTLE);

        case AutoState::SEEK:
            if (opp_valid)
                return _steer(opp_bearing, 0.6f, SEEK_THROTTLE);   // head toward opponent
            else
                return _steer(_searchDir,  0.3f, SEEK_THROTTLE);   // slowly circle to search

        case AutoState::ATTACK:
            // Commit — full throttle, steer toward last known bearing.
            return _steer(opp_valid ? opp_bearing : _searchDir,
                          1.0f, ATTACK_THROTTLE);

        case AutoState::EVADE:
            // Drive directly away from the wall.
            return _steer(wall_bearing + PI, 0.8f, SEEK_THROTTLE);
    }

    return _steer(0.0f, 0.0f, 0.0f);  // unreachable
}


// ─── Private helpers ──────────────────────────────────────────────────────────

void Autonomy::_changeState(AutoState s) {
    if (s == _state) return;
    const char* names[] = { "IDLE", "SPIN_UP", "SEEK", "ATTACK", "EVADE" };
    Serial.printf("[Auto] %s → %s\n", names[(int)_state], names[(int)s]);
    _state   = s;
    _stateMs = millis();
}

DriveCommand Autonomy::_steer(float bearing, float mag, float throttle) {
    mag      = fmaxf(0.0f, fminf(1.0f, mag));
    throttle = fmaxf(0.0f, fminf(1.0f, throttle));

    DriveCommand cmd = {};
    cmd.isTank = false;
    cmd.ch1_us = CH1_MID + sinf(bearing) * mag * CH1_HALF;   // lateral
    cmd.ch2_us = CH2_MID + cosf(bearing) * mag * CH2_HALF;   // forward
    cmd.ch3_us = CH3_LO  + throttle * (CH3_HI - CH3_LO);    // spin
    return cmd;
}

DriveCommand Autonomy::_rcPassthrough(const uint16_t* rc) {
    DriveCommand cmd = {};
    cmd.isTank = false;
    cmd.ch1_us = (float)rc[0];
    cmd.ch2_us = (float)rc[1];
    cmd.ch3_us = (float)rc[2];
    return cmd;
}

float Autonomy::_mapUs(float us, float in_lo, float in_hi) {
    float mid  = (in_lo + in_hi) * 0.5f;
    float half = (in_hi - in_lo) * 0.5f;
    return fmaxf(-1.0f, fminf(1.0f, (us - mid) / half));
}

void Autonomy::printState() const {
    static const char* modeNames[]  = { "TANK", "MELTY", "ASSISTED", "AUTO" };
    static const char* stateNames[] = { "IDLE", "SPIN_UP", "SEEK", "ATTACK", "EVADE" };
    Serial.printf("[Autonomy] mode=%-8s  state=%s\n",
                  modeNames[(int)_mode], stateNames[(int)_state]);
}
