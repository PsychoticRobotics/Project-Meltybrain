#include "IRArena.h"
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════════
// IRArenaTracker — beacon-free arena localization
// ═══════════════════════════════════════════════════════════════════════════════
//
// Algorithm overview (per revolution):
//
//   1. Collect hits from the TSOP receiver as the robot spins.
//   2. At the revolution boundary (angle wraps past 2π):
//      a. Match each hit to the nearest existing wall slot by bearing.
//      b. Update matched walls via EMA.  Reset persistence if bearing
//         shifted too much (object is moving → not a wall).
//      c. Create new candidate slots for unmatched hits (if room).
//      d. Report remaining unmatched hits as opponent candidates.
//      e. Decay/remove slots that haven't been seen recently.
//      f. Estimate heading from the wall bearing pattern (90° grid fit).
//      g. Estimate position from angular-width ratios of opposite pairs.


// ─── Setup ───────────────────────────────────────────────────────────────────

void IRArenaTracker::init(uint8_t emitterPin, uint8_t receiverPin) {
    _emitterPin = emitterPin;
    pinMode(_emitterPin, OUTPUT);
    digitalWrite(_emitterPin, LOW);
    analogWriteFrequency(_emitterPin, IR_SWEEP_FREQ);

    _receiver.init(receiverPin);

    for (int i = 0; i < ARENA_MAX_WALLS; i++) {
        _walls[i] = {};
    }

    Serial.println("[Arena] IR arena tracker initialised.");
}

void IRArenaTracker::setSquareArena(float sideLength_m) {
    _arenaSize = sideLength_m;
    _arenaConfigured = true;
    Serial.printf("[Arena] Arena set to %.2f m (%.1f ft) square.\n",
                  sideLength_m, sideLength_m / 0.3048f);
}


// ─── Emitter control ─────────────────────────────────────────────────────────

void IRArenaTracker::enable() {
    if (!_enabled) {
        // 50% duty cycle: maximise pulse energy within GPIO current limits.
        analogWrite(_emitterPin, 128);
        _enabled = true;
    }
}

void IRArenaTracker::disable() {
    if (_enabled) {
        analogWrite(_emitterPin, 0);
        _enabled = false;
    }
}


// ─── Main update ─────────────────────────────────────────────────────────────

void IRArenaTracker::update(uint32_t t_us, float currentAngle, float omega) {
    if (!_enabled) return;

    // ── Collect hits from the receiver ───────────────────────────────────────
    if (_receiver.hasPulse()) {
        // Back-interpolate bearing to pulse midpoint (same technique as IRSweep).
        float elapsed_s = (float)(t_us - _receiver.getMidTime()) * 1e-6f;
        if (elapsed_s < 0.0f) elapsed_s = 0.0f;
        float bearing = _wrapAngle(currentAngle - omega * elapsed_s);

        // Angular width = pulse duration converted to radians at current omega.
        // Wider return → closer to the reflecting surface.
        float pulseWidth_us = (float)(_receiver.getRiseTime() - _receiver.getFallTime());
        float angularWidth  = fabsf(omega) * pulseWidth_us * 1e-6f;

        _receiver.clearPulse();

        // Only store hits when spinning fast enough for useful resolution.
        if (_revHitCount < ARENA_HITS_PER_REV && omega >= ARENA_MIN_OMEGA) {
            _revHits[_revHitCount++] = { bearing, angularWidth };
        }
    }

    // ── Detect revolution boundary ───────────────────────────────────────────
    // The estimator's angle wraps from near 2π back to near 0 once per rev.
    // Guard against false triggers from heading corrections by requiring at
    // least half the expected revolution period to have elapsed.
    if (_prevAngle > 4.5f && currentAngle < 1.5f && omega >= ARENA_MIN_OMEGA) {
        uint32_t elapsed_us = t_us - _lastRevUs;
        float    expectedPeriod_us = (2.0f * PI / omega) * 1e6f;

        if (elapsed_us > (uint32_t)(expectedPeriod_us * 0.5f)) {
            _processRevolution(omega);
            _revHitCount = 0;
            _revCount++;
            _lastRevUs = t_us;
        }
    }
    _prevAngle = currentAngle;
}


// ─── Process one revolution ──────────────────────────────────────────────────

void IRArenaTracker::_processRevolution(float omega) {
    if (_revHitCount == 0) return;

    // ── Match new hits to existing wall slots ────────────────────────────────
    bool hitMatched[ARENA_HITS_PER_REV]  = {};
    bool wallSeen[ARENA_MAX_WALLS]       = {};

    for (int h = 0; h < _revHitCount; h++) {
        float bestDist = ARENA_WALL_MATCH_RAD;
        int   bestSlot = -1;

        for (int w = 0; w < _slotCount; w++) {
            float d = _angleDist(_revHits[h].bearing, _walls[w].bearing);
            if (d < bestDist) {
                bestDist = d;
                bestSlot = w;
            }
        }

        if (bestSlot >= 0 && !wallSeen[bestSlot]) {
            ArenaWall& wall = _walls[bestSlot];

            // Compute bearing delta (handles wrap-around).
            float delta = _revHits[h].bearing - wall.bearing;
            if (delta >  PI) delta -= 2.0f * PI;
            if (delta < -PI) delta += 2.0f * PI;

            // If the bearing jumped too much, this slot is tracking a moving
            // object — reset its persistence so it doesn't become a wall.
            if (fabsf(delta) > ARENA_WALL_MAX_DRIFT) {
                wall.persistence = 0;
                wall.isWall      = false;
            }

            // EMA update for bearing and angular width.
            wall.bearing      = _wrapAngle(wall.bearing + ARENA_EMA_ALPHA * delta);
            wall.angularWidth += ARENA_EMA_ALPHA * (_revHits[h].angularWidth - wall.angularWidth);

            wall.persistence++;
            wall.missCount = 0;
            if (wall.persistence >= ARENA_WALL_MIN_PERSISTENCE) {
                wall.isWall = true;
            }

            wallSeen[bestSlot] = true;
            hitMatched[h]      = true;
        }
    }

    // ── Create new candidate slots for unmatched hits ────────────────────────
    for (int h = 0; h < _revHitCount; h++) {
        if (hitMatched[h]) continue;

        if (_slotCount < ARENA_MAX_WALLS) {
            _walls[_slotCount] = {
                .bearing      = _revHits[h].bearing,
                .angularWidth = _revHits[h].angularWidth,
                .persistence  = 1,
                .missCount    = 0,
                .isWall       = false
            };
            _slotCount++;
            hitMatched[h] = true;
        }
    }

    // ── Report remaining unmatched hits as opponents ─────────────────────────
    _opponentHitCount = 0;
    for (int h = 0; h < _revHitCount; h++) {
        if (!hitMatched[h] && _opponentHitCount < ARENA_MAX_OPPONENT_HITS) {
            _opponentAngles[_opponentHitCount++] = _revHits[h].bearing;
        }
    }

    // ── Decay / remove stale slots ───────────────────────────────────────────
    for (int w = 0; w < _slotCount; w++) {
        if (!wallSeen[w]) {
            _walls[w].missCount++;
            if (_walls[w].missCount >= ARENA_WALL_DECAY_REVS) {
                // Remove by shifting remaining slots down.
                for (int j = w; j < _slotCount - 1; j++) {
                    _walls[j] = _walls[j + 1];
                }
                _slotCount--;
                w--;   // re-check this index after shift
            }
        }
    }

    // ── Heading and position estimation ──────────────────────────────────────
    _estimateHeading();
    if (_arenaConfigured) {
        _estimatePosition();
    }
}


// ─── Heading correction from wall pattern ────────────────────────────────────
//
// In a square arena the 4 wall bearings lie on a 90° (π/2) grid.  We find the
// rotation R of that grid by taking the circular mean of all classified wall
// bearings modulo π/2.
//
// First calibration: save R as the reference.
// Subsequent revolutions: error = R_measured − R_reference.
//
// Trick: multiply each bearing by 4 so the π/2 periodicity maps to 2π, take
// the standard circular mean, then divide by 4.

void IRArenaTracker::_estimateHeading() {
    int wallCount = 0;
    float sinSum = 0.0f, cosSum = 0.0f;

    for (int i = 0; i < _slotCount; i++) {
        if (!_walls[i].isWall) continue;
        wallCount++;

        float b4 = _walls[i].bearing * 4.0f;
        sinSum += sinf(b4);
        cosSum += cosf(b4);
    }

    if (wallCount < 2) return;

    float arenaRot = atan2f(sinSum, cosSum) / 4.0f;
    if (arenaRot < 0.0f) arenaRot += PI / 2.0f;

    if (!_arenaRotationCalibrated) {
        _arenaRotation = arenaRot;
        _arenaRotationCalibrated = true;
        Serial.printf("[Arena] Arena orientation calibrated: %.1f deg\n",
                      arenaRot * (180.0f / PI));
    } else {
        float error = arenaRot - _arenaRotation;
        // Wrap to [−π/4, π/4] — working modulo π/2.
        while (error >  PI / 4.0f) error -= PI / 2.0f;
        while (error < -PI / 4.0f) error += PI / 2.0f;

        _headingError  = error;
        _hasHeadingFix = (fabsf(error) > 0.005f);
    }
}


// ─── Position estimation from angular-width ratios ───────────────────────────
//
// For two walls that are roughly opposite (bearings ≈ 180° apart):
//
//   closer wall → wider angular return
//
//   distance_to_A / distance_to_B  ≈  width_B / width_A   (approximate)
//
//   d_A + d_B = L  (arena side length)
//
//   position_along_axis  =  L/2 × (w_A − w_B) / (w_A + w_B)
//                           positive → closer to wall A
//
// We decompose this scalar position into (x, y) using the bearing of wall A:
//
//   Δx = pos × cos(bearing_A)
//   Δy = pos × sin(bearing_A)
//
// Two orthogonal pairs → full (x, y).

void IRArenaTracker::_estimatePosition() {
    // Collect classified walls.
    int wallIdx[ARENA_MAX_WALLS];
    int nWalls = 0;
    for (int i = 0; i < _slotCount; i++) {
        if (_walls[i].isWall && nWalls < ARENA_MAX_WALLS) {
            wallIdx[nWalls++] = i;
        }
    }
    if (nWalls < 2) { _hasPosition = false; return; }

    // Find opposite pairs (bearing difference within 30° of 180°).
    bool paired[ARENA_MAX_WALLS] = {};
    float sumX = 0.0f, sumY = 0.0f;
    int pairCount = 0;

    for (int i = 0; i < nWalls && pairCount < 2; i++) {
        if (paired[i]) continue;
        for (int j = i + 1; j < nWalls; j++) {
            if (paired[j]) continue;
            float d = _angleDist(_walls[wallIdx[i]].bearing,
                                 _walls[wallIdx[j]].bearing);
            if (fabsf(d - PI) < 0.52f) {   // within ~30° of opposite
                float wA = _walls[wallIdx[i]].angularWidth;
                float wB = _walls[wallIdx[j]].angularWidth;
                if (wA + wB < 0.001f) continue;

                // Scalar position along axis toward wall A.
                float pos = (_arenaSize / 2.0f) * (wA - wB) / (wA + wB);
                float bearing = _walls[wallIdx[i]].bearing;

                sumX += pos * cosf(bearing);
                sumY += pos * sinf(bearing);

                paired[i] = paired[j] = true;
                pairCount++;
                break;
            }
        }
    }

    if (pairCount > 0) {
        _posX = sumX;
        _posY = sumY;
        _hasPosition = true;
    }
}


// ─── Accessors ───────────────────────────────────────────────────────────────

int IRArenaTracker::getWallCount() const {
    int count = 0;
    for (int i = 0; i < _slotCount; i++) {
        if (_walls[i].isWall) count++;
    }
    return count;
}

const ArenaWall& IRArenaTracker::getWall(int index) const {
    static const ArenaWall empty = {};
    int count = 0;
    for (int i = 0; i < _slotCount; i++) {
        if (_walls[i].isWall) {
            if (count == index) return _walls[i];
            count++;
        }
    }
    return empty;
}

float IRArenaTracker::getNearestWallDist() const {
    if (!_hasPosition || !_arenaConfigured || !_arenaRotationCalibrated)
        return 999.0f;

    float L2 = _arenaSize / 2.0f;

    // Rotate position into arena-wall-aligned coordinates.
    float c  = cosf(_arenaRotation), s = sinf(_arenaRotation);
    float xw =  _posX * c + _posY * s;
    float yw = -_posX * s + _posY * c;

    // 4 perpendicular wall distances.
    float dists[4] = {
        L2 - xw,    // wall in +X direction
        L2 + xw,    // wall in −X direction
        L2 - yw,    // wall in +Y direction
        L2 + yw,    // wall in −Y direction
    };

    float minDist = dists[0];
    for (int i = 1; i < 4; i++) {
        if (dists[i] < minDist) minDist = dists[i];
    }
    return (minDist > 0.0f) ? minDist : 0.0f;
}

float IRArenaTracker::getNearestWallBearing() const {
    // The wall with the largest angular width is the closest.
    float maxWidth = 0.0f;
    float bearing  = 0.0f;
    for (int i = 0; i < _slotCount; i++) {
        if (_walls[i].isWall && _walls[i].angularWidth > maxWidth) {
            maxWidth = _walls[i].angularWidth;
            bearing  = _walls[i].bearing;
        }
    }
    return bearing;
}

float IRArenaTracker::getDistanceInDirection(float bearing) const {
    if (!_hasPosition || !_arenaConfigured || !_arenaRotationCalibrated)
        return 999.0f;

    float L2 = _arenaSize / 2.0f;

    // Rotate position and ray direction into arena-wall-aligned coordinates.
    float c  = cosf(_arenaRotation), s = sinf(_arenaRotation);
    float px =  _posX * c + _posY * s;
    float py = -_posX * s + _posY * c;
    float dx =  cosf(bearing) * c + sinf(bearing) * s;
    float dy = -cosf(bearing) * s + sinf(bearing) * c;

    // Ray–AABB intersection: find the nearest wall in the forward direction.
    float tMin = 999.0f;

    if (fabsf(dx) > 1e-6f) {
        float t1 = ( L2 - px) / dx;   // +X wall
        float t2 = (-L2 - px) / dx;   // −X wall
        if (t1 > 0.0f && t1 < tMin) tMin = t1;
        if (t2 > 0.0f && t2 < tMin) tMin = t2;
    }
    if (fabsf(dy) > 1e-6f) {
        float t3 = ( L2 - py) / dy;   // +Y wall
        float t4 = (-L2 - py) / dy;   // −Y wall
        if (t3 > 0.0f && t3 < tMin) tMin = t3;
        if (t4 > 0.0f && t4 < tMin) tMin = t4;
    }

    return tMin;
}

float IRArenaTracker::getOpponentHitAngle(int index) const {
    if (index < 0 || index >= _opponentHitCount) return 0.0f;
    return _opponentAngles[index];
}


// ─── Diagnostics ─────────────────────────────────────────────────────────────

void IRArenaTracker::printWalls() const {
    Serial.printf("[Arena] %d slots, %d classified walls, %lu revolutions\n",
                  _slotCount, getWallCount(), (unsigned long)_revCount);
    for (int i = 0; i < _slotCount; i++) {
        Serial.printf("  [%d] %5.1f deg  w=%.3f rad  persist=%u  miss=%u  %s\n",
                      i,
                      _walls[i].bearing * (180.0f / PI),
                      _walls[i].angularWidth,
                      _walls[i].persistence,
                      _walls[i].missCount,
                      _walls[i].isWall ? "WALL" : "candidate");
    }
    if (_hasPosition) {
        Serial.printf("[Arena] Position: (%.3f, %.3f) m   nearest wall: %.3f m\n",
                      _posX, _posY, getNearestWallDist());
    }
    if (_arenaRotationCalibrated) {
        Serial.printf("[Arena] Arena rotation: %.1f deg\n",
                      _arenaRotation * (180.0f / PI));
    }
}


// ─── Utility ─────────────────────────────────────────────────────────────────

float IRArenaTracker::_wrapAngle(float a) const {
    a = fmodf(a, 2.0f * PI);
    if (a < 0.0f) a += 2.0f * PI;
    return a;
}

float IRArenaTracker::_angleDist(float a, float b) const {
    float d = fabsf(a - b);
    return (d > PI) ? (2.0f * PI - d) : d;
}
