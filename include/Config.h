//
// Created by Atharv Goel on 9/28/25.
// Updated last by Atharv Goel on 10/02/25.
//

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// IR mode:  0 = external beacons (IRBeaconTracker + IRSweep)
//           1 = beacon-free arena tracking (IRArenaTracker)
// In arena mode pins 2 and 3 are freed up — no external beacons needed.
#define IR_MODE  1

// Arena size in metres — only used when IR_MODE == 1.
// Common sizes: 8 ft = 2.44 m, 12 ft = 3.66 m, 16 ft = 4.88 m.
#define ARENA_SIZE_M  2.44f

// ─── Control mode selection ───────────────────────────────────────────────────
// Wire a 3- or 4-position switch to this RC channel (0-based index).
// Default: CH5 (channels[4]).  Adjust thresholds to match your radio.
//
//   µs < TANK     → TANK      (non-spinning differential drive)
//   µs < MELTY    → MELTY     (RC-controlled melty translation)
//   µs < ASSISTED → ASSISTED  (melty + wall avoidance + opponent seek)
//   µs ≥ ASSISTED → AUTO      (full autonomous state machine)
//
// On a 3-position switch (~1000 / ~1500 / ~2000 µs):
//   low=TANK  mid=MELTY  high=ASSISTED   (AUTO unreachable — set to taste)
#define MODE_SELECT_CH           4       // 0-based channel index (CH5)
#define MODE_THRESHOLD_TANK   1200       // µs: below this → TANK
#define MODE_THRESHOLD_MELTY  1450       // µs: below this → MELTY
#define MODE_THRESHOLD_ASST   1700       // µs: below this → ASSISTED, above → AUTO

// ─── Autonomy tuning ──────────────────────────────────────────────────────────
#define SPIN_UP_OMEGA     150.0f   // rad/s — "at operating speed" threshold
#define MIN_DRIVE_OMEGA    30.0f   // rad/s — below this, melty translation is unreliable
#define WALL_AVOID_DIST    0.4f    // m — begin wall avoidance within this distance
#define WALL_SAFE_DIST     0.7f    // m — wall considered safe again beyond this
#define OPP_SEEK_GAIN      0.4f    // ASSISTED: weight of opponent-seek nudge
#define WALL_AVOID_GAIN    0.5f    // ASSISTED: weight of wall-repulsion nudge
#define ATTACK_THROTTLE    1.0f    // AUTO: spin throttle [0-1] in ATTACK state
#define SEEK_THROTTLE      0.85f   // AUTO: spin throttle [0-1] in SEEK/EVADE states

#endif //MAIN_CONFIG_H
