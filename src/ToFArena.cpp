#include "ToFArena.h"

// ─── Sensor driver: Benewake TFMini-S on Serial2 ──────────────────────────────
//
// The TFMini-S streams 9-byte frames continuously:
//   [0] 0x59         header byte 1
//   [1] 0x59         header byte 2
//   [2] dist_low     distance in cm, low byte
//   [3] dist_high    distance in cm, high byte
//   [4] strength_low signal strength, low byte
//   [5] strength_high signal strength, high byte
//   [6] temp_low     temperature, low byte  (unused here)
//   [7] temp_high    temperature, high byte (unused here)
//   [8] checksum     sum of bytes [0..7] truncated to 8 bits
//
// Frames arrive back-to-back over UART. The parser below buffers bytes as they
// arrive and emits a reading whenever it has a complete frame with a valid
// checksum. Non-blocking — returns immediately if no complete frame is ready.

static constexpr uint8_t TFMINI_FRAME_SIZE = 9;
static constexpr uint8_t TFMINI_HEADER     = 0x59;

static uint8_t _rxBuf[TFMINI_FRAME_SIZE];
static uint8_t _rxIdx = 0;

static bool _sensorInit() {
    Serial2.begin(TFMINI_BAUD);
    // TFMini-S has no init handshake — it begins streaming as soon as power and
    // UART are up. We always return true; use validBinCount() after a few
    // revolutions to confirm data is actually arriving.
    return true;
}

// Returns true and fills dist_m + strength once per complete valid frame.
// dist_m is in metres; strength is 0–65535 (Benewake's 16-bit signal quality).
static bool _sensorRead(float* dist_m, uint16_t* strength) {
    while (Serial2.available()) {
        uint8_t b = (uint8_t)Serial2.read();

        // ── Resync on the two-byte header ─────────────────────────────────────
        if (_rxIdx == 0) {
            if (b == TFMINI_HEADER) _rxBuf[_rxIdx++] = b;
            continue;
        }
        if (_rxIdx == 1) {
            if (b == TFMINI_HEADER) {
                _rxBuf[_rxIdx++] = b;
            } else {
                _rxIdx = 0;   // false start — drop and resync
            }
            continue;
        }

        // ── Accumulate payload + checksum ─────────────────────────────────────
        _rxBuf[_rxIdx++] = b;

        if (_rxIdx == TFMINI_FRAME_SIZE) {
            uint8_t sum = 0;
            for (int i = 0; i < TFMINI_FRAME_SIZE - 1; i++) sum += _rxBuf[i];
            uint8_t expected = _rxBuf[TFMINI_FRAME_SIZE - 1];

            _rxIdx = 0;   // ready for the next frame regardless of CRC outcome

            if (sum == expected) {
                uint16_t dist_cm = (uint16_t)_rxBuf[2] | ((uint16_t)_rxBuf[3] << 8);
                uint16_t str     = (uint16_t)_rxBuf[4] | ((uint16_t)_rxBuf[5] << 8);
                *dist_m   = dist_cm * 0.01f;
                *strength = str;
                return true;
            }
            // bad checksum — fall through and keep reading
        }
    }
    return false;
}

// ─── ToFArenaMapper ───────────────────────────────────────────────────────────

bool ToFArenaMapper::init() {
    _sensorReady = _sensorInit();
    Serial.println("[ToF] TFMini-S UART initialised on Serial2 @ "
                   + String(TFMINI_BAUD) + " baud.");
    return _sensorReady;
}

// ─── One-time configuration ──────────────────────────────────────────────────
// Sends the TFMini-S "set frame rate = 1000 Hz" command followed by the
// "save settings" command, persisting the change to the sensor's flash.
// Call once with the sensor connected, then re-comment the call in setup().
// See the header for protocol details.

void ToFArenaMapper::configure1000Hz() {
    if (!_sensorReady) {
        Serial.println("[ToF] configure1000Hz: sensor not initialised, abort.");
        return;
    }

    Serial.println("[ToF] Configuring TFMini-S for 1000 Hz output...");

    // Set frame rate = 1000 Hz (0x03E8 little-endian)
    static const uint8_t cmd_rate[]  = { 0x5A, 0x06, 0x03, 0xE8, 0x03, 0x48 };
    // Save settings to sensor flash
    static const uint8_t cmd_save[]  = { 0x5A, 0x04, 0x11, 0x6F };

    Serial2.write(cmd_rate, sizeof(cmd_rate));
    Serial2.flush();
    delay(100);

    Serial2.write(cmd_save, sizeof(cmd_save));
    Serial2.flush();
    delay(100);

    // Drain any acknowledgement bytes so they don't trip up the frame parser.
    while (Serial2.available()) Serial2.read();
    _rxIdx = 0;

    Serial.println("[ToF] TFMini-S now configured to 1000 Hz, saved to flash.");
}

void ToFArenaMapper::update(float angle_rad, float omega_rad_s) {
    if (!_sensorReady) return;

    float    dist_m;
    uint16_t strength;
    if (!_readSensor(&dist_m, &strength)) return;   // no fresh frame this loop

    // ── Reject obviously bad readings ─────────────────────────────────────────
    if (strength < TOF_MIN_STRENGTH) return;
    if (dist_m   < TOF_MIN_RANGE_M)  return;
    if (dist_m   > TOF_MAX_RANGE_M)  return;

    // ── Pipeline delay compensation ───────────────────────────────────────────
    // The sensor triggered this measurement ~TOF_PIPELINE_MS ago.
    // Back-rotate the angle to where the robot was when the pulse was sent.
    float meas_angle = angle_rad - omega_rad_s * (TOF_PIPELINE_MS * 0.001f);
    // Normalise to [0, 2π)
    meas_angle = fmodf(meas_angle, 2.0f * (float)M_PI);
    if (meas_angle < 0.0f) meas_angle += 2.0f * (float)M_PI;

    _addReading(meas_angle, dist_m);
}

// ─── Private ─────────────────────────────────────────────────────────────────

void ToFArenaMapper::_addReading(float angle_rad, float dist_m) {
    int bin = angleToBin(angle_rad);

    if (!_bins[bin].valid) {
        // First reading for this bin — accept as wall baseline.
        _bins[bin].dist       = dist_m;
        _bins[bin].valid      = true;
        _bins[bin].isOpponent = false;
        return;
    }

    float established = _bins[bin].dist;
    float delta       = established - dist_m;   // positive → new reading is closer

    if (delta > TOF_OPP_THRESHOLD) {
        // New reading is substantially closer than the established wall distance.
        // Flag as a potential opponent return; do NOT update the wall EMA so the
        // wall estimate stays stable while the opponent is in the way.
        _bins[bin].isOpponent = true;
    } else {
        // Normal wall reading — fold into the EMA and clear any opponent flag.
        _bins[bin].dist       = (1.0f - TOF_EMA_ALPHA) * established
                              + TOF_EMA_ALPHA * dist_m;
        _bins[bin].isOpponent = false;
    }
}

bool ToFArenaMapper::_readSensor(float* dist_m, uint16_t* strength) {
    return _sensorRead(dist_m, strength);
}

// ─── Queries ─────────────────────────────────────────────────────────────────

float ToFArenaMapper::binDistance(int bin) const {
    if (bin < 0 || bin >= TOF_NUM_BINS) return TOF_MAX_RANGE_M;
    return _bins[bin].valid ? _bins[bin].dist : TOF_MAX_RANGE_M;
}

float ToFArenaMapper::distanceAtAngle(float angle_rad) const {
    int   binA = angleToBin(angle_rad);
    int   binB = (binA + 1) % TOF_NUM_BINS;

    if (!_bins[binA].valid && !_bins[binB].valid) return TOF_MAX_RANGE_M;
    if (!_bins[binA].valid) return _bins[binB].dist;
    if (!_bins[binB].valid) return _bins[binA].dist;

    // Linear interpolation between the two surrounding bin centres.
    float centreA = binToAngle(binA);
    float t       = (angle_rad - centreA) / TOF_BIN_RAD;
    t             = fmaxf(0.0f, fminf(1.0f, t));
    return (1.0f - t) * _bins[binA].dist + t * _bins[binB].dist;
}

float ToFArenaMapper::nearestBearing() const {
    float best     = TOF_MAX_RANGE_M;
    int   bestBin  = 0;
    for (int i = 0; i < TOF_NUM_BINS; i++) {
        if (_bins[i].valid && _bins[i].dist < best) {
            best    = _bins[i].dist;
            bestBin = i;
        }
    }
    return binToAngle(bestBin);
}

float ToFArenaMapper::nearestDistance() const {
    float best = TOF_MAX_RANGE_M;
    for (int i = 0; i < TOF_NUM_BINS; i++) {
        if (_bins[i].valid && _bins[i].dist < best)
            best = _bins[i].dist;
    }
    return best;
}

int ToFArenaMapper::validBinCount() const {
    int n = 0;
    for (int i = 0; i < TOF_NUM_BINS; i++)
        if (_bins[i].valid) n++;
    return n;
}

void ToFArenaMapper::clear() {
    for (int i = 0; i < TOF_NUM_BINS; i++)
        _bins[i] = Bin{};
}

// ─── Static helpers ───────────────────────────────────────────────────────────

int ToFArenaMapper::angleToBin(float angle_rad) {
    // Normalise to [0, 2π) then divide by bin width.
    angle_rad = fmodf(angle_rad, 2.0f * (float)M_PI);
    if (angle_rad < 0.0f) angle_rad += 2.0f * (float)M_PI;
    int bin = (int)(angle_rad / TOF_BIN_RAD);
    // clamp — should never be needed after normalisation but guards float edge cases
    if (bin >= TOF_NUM_BINS) bin = TOF_NUM_BINS - 1;
    return bin;
}

float ToFArenaMapper::binToAngle(int bin) {
    return (bin + 0.5f) * TOF_BIN_RAD;   // centre of the bin
}

// ─── Debug ───────────────────────────────────────────────────────────────────

void ToFArenaMapper::printMap() const {
    Serial.printf("[ToF] Polar map (%d/%d bins valid):\n", validBinCount(), TOF_NUM_BINS);
    for (int i = 0; i < TOF_NUM_BINS; i++) {
        if (!_bins[i].valid) continue;
        Serial.printf("  bin %2d  %5.1f°  dist=%.3f m%s\n",
            i,
            binToAngle(i) * (180.0f / (float)M_PI),
            _bins[i].dist,
            _bins[i].isOpponent ? "  [OPP]" : "");
    }
}
