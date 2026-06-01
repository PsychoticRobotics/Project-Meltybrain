#include "ToFArena.h"
#include <Wire.h>

// ─── Sensor driver ────────────────────────────────────────────────────────────
//
// This section is the only place that touches the TMF8801 hardware. Swap out
// just this section if you change libraries or sensor family.
//
// ── TODO: fill in with your chosen TMF8801 library ───────────────────────────
//
// When you have the hardware, install the library and replace the three
// functions below (_sensorInit, _sensorRead, and any globals) with real calls.
//
// ams-OSRAM maintain an Arduino driver here:
//   https://github.com/ams-OSRAM-Group/tmf8x0x-arduino-driver
//
// The rest of this file (polar map, EMA, opponent detection) does not change.
// ─────────────────────────────────────────────────────────────────────────────

static bool _sensorInit() {
    // TODO: initialise TMF8801 over I2C and start free-running measurements.
    // Return true on success, false if sensor not found.
    return false;
}

// Returns true and fills dist_m + confidence if a fresh reading is available.
// dist_m is in metres; confidence is 0–255.
static bool _sensorRead(float* dist_m, uint8_t* confidence) {
    // TODO: check if a new measurement is ready and fill dist_m / confidence.
    (void)dist_m; (void)confidence;
    return false;
}

// ─── ToFArenaMapper ───────────────────────────────────────────────────────────

bool ToFArenaMapper::init() {
    _sensorReady = _sensorInit();
    if (!_sensorReady) {
        Serial.println("[ToF] Sensor not found — check wiring and I2C address.");
    } else {
        Serial.println("[ToF] TMF8801 initialised.");
    }
    return _sensorReady;
}

void ToFArenaMapper::update(float angle_rad, float omega_rad_s) {
    if (!_sensorReady) return;

    float dist_m;
    uint8_t conf;
    if (!_readSensor(&dist_m, &conf)) return;   // no fresh reading this loop

    // ── Reject obviously bad readings ─────────────────────────────────────────
    if (conf < TOF_MIN_CONF)       return;
    if (dist_m < TOF_MIN_RANGE_M)  return;
    if (dist_m > TOF_MAX_RANGE_M)  return;

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

bool ToFArenaMapper::_readSensor(float* dist_m, uint8_t* confidence) {
    return _sensorRead(dist_m, confidence);
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
