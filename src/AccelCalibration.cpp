#include "AccelCalibration.h"
#include <EEPROM.h>

// ─── EEPROM layout ───────────────────────────────────────────────────────────
//
//  Offset  Size   Content
//  ──────  ─────  ────────────────────────────────────────────────────────────
//  0       1 B    Sentinel byte A (ACCEL_CAL_SENTINEL_A = 0xAC)
//  1       1 B    Sentinel byte B (ACCEL_CAL_SENTINEL_B = 0xC1)
//  2       72 B   AxisCal for sensor 0 (top accelerometer)
//  74      72 B   AxisCal for sensor 1 (bottom accelerometer)
//  ──────  ─────
//  Total   146 B  (Teensy 4.1 provides 4 284 B of emulated EEPROM)
//
// sizeof(AxisCal) = 4 (zeroOffset) + 8×8 (table) + 1 (len) + 1 (evictPos)
//                + 2 (ARM 4-byte alignment padding) = 72 bytes.
// If the compiler reports a different size the ADDR_CAL1 constant below will
// automatically adjust via sizeof().

static constexpr uint16_t ADDR_SENTINEL = ACCEL_CAL_EEPROM_BASE;
static constexpr uint16_t ADDR_CAL0     = ADDR_SENTINEL + 2;
static constexpr uint16_t ADDR_CAL1     = ADDR_CAL0 + (uint16_t)sizeof(AxisCal);

// ─── Persistence ─────────────────────────────────────────────────────────────

void AccelCalibrationManager::load() {
    uint8_t s0 = EEPROM.read(ADDR_SENTINEL);
    uint8_t s1 = EEPROM.read(ADDR_SENTINEL + 1);

    if (s0 == ACCEL_CAL_SENTINEL_A && s1 == ACCEL_CAL_SENTINEL_B) {
        EEPROM.get(ADDR_CAL0, _cal[0]);
        EEPROM.get(ADDR_CAL1, _cal[1]);

        // Bounds-check lengths in case of partial EEPROM corruption.
        _cal[0].len = min(_cal[0].len, (uint8_t)ACCEL_CAL_POINTS);
        _cal[1].len = min(_cal[1].len, (uint8_t)ACCEL_CAL_POINTS);

        Serial.printf("[AccelCal] Loaded — S0: zero=%.3fg, %u pts | S1: zero=%.3fg, %u pts\n",
                      _cal[0].zeroOffset, _cal[0].len,
                      _cal[1].zeroOffset, _cal[1].len);
    } else {
        reset();
        Serial.printf("[AccelCal] EEPROM blank or stale (sentinel %02X %02X) — neutral defaults.\n",
                      s0, s1);
    }
}

void AccelCalibrationManager::save() {
    // EEPROM.put() handles multi-byte writes correctly — unlike EEPROM.write()
    // which only writes a single byte and would corrupt float fields.
    EEPROM.write(ADDR_SENTINEL,     ACCEL_CAL_SENTINEL_A);
    EEPROM.write(ADDR_SENTINEL + 1, ACCEL_CAL_SENTINEL_B);
    EEPROM.put(ADDR_CAL0, _cal[0]);
    EEPROM.put(ADDR_CAL1, _cal[1]);
    _dirty = false;

    Serial.printf("[AccelCal] Saved to EEPROM (%u + %u correction pts).\n",
                  _cal[0].len, _cal[1].len);
}

void AccelCalibrationManager::reset() {
    for (uint8_t i = 0; i < 2; i++) {
        _cal[i].zeroOffset = 0.0f;
        _cal[i].len        = 0;
        _cal[i].evictPos   = 0;
        for (uint8_t j = 0; j < ACCEL_CAL_POINTS; j++) {
            _cal[i].table[j] = {0.0f, 0.0f};
        }
    }
    _workingFactor = 0.0f;
    _dirty = false;
}

// ─── Runtime correction ──────────────────────────────────────────────────────

float AccelCalibrationManager::apply(uint8_t sensorIdx, float rawG) const {
    const AxisCal& c = _cal[sensorIdx & 1];

    // Step 1: remove DC bias.
    float g = rawG - c.zeroOffset;

    // Step 2: apply piecewise-linear gain correction.
    if (c.len > 0) {
        g *= (1.0f + interpolate(c, fabsf(g)));
    }

    return g;
}

// ─── Calibration session ─────────────────────────────────────────────────────

void AccelCalibrationManager::captureZero(uint8_t sensorIdx, float mean) {
    _cal[sensorIdx & 1].zeroOffset = mean;
    _dirty = true;
    Serial.printf("[AccelCal] Zero offset S%u = %+.4f g\n", sensorIdx, mean);
}

void AccelCalibrationManager::commitPoint(float gSensor0, float gSensor1) {
    // Subtract each sensor's own zero offset before storing the G reference
    // so the table is indexed in the same "corrected-DC" space that apply() uses.
    float g0 = fabsf(gSensor0 - _cal[0].zeroOffset);
    float g1 = fabsf(gSensor1 - _cal[1].zeroOffset);

    sortedInsert(0, g0, _workingFactor);
    sortedInsert(1, g1, _workingFactor);
    _dirty = true;

    Serial.printf("[AccelCal] Point committed — factor=%+.4f  S0 @ %.2fg  S1 @ %.2fg\n",
                  _workingFactor, g0, g1);
    printTable();
}

void AccelCalibrationManager::clearTable(uint8_t sensorIdx) {
    _cal[sensorIdx & 1].len      = 0;
    _cal[sensorIdx & 1].evictPos = 0;
    _dirty = true;
}

void AccelCalibrationManager::clearAllTables() {
    clearTable(0);
    clearTable(1);
    Serial.println("[AccelCal] All correction tables cleared (zero offsets preserved).");
}

// ─── Diagnostics ─────────────────────────────────────────────────────────────

void AccelCalibrationManager::printTable() const {
    for (uint8_t s = 0; s < 2; s++) {
        Serial.printf("  [S%u | zero=%+.3fg | %u/%u pts]",
                      s, _cal[s].zeroOffset, _cal[s].len, ACCEL_CAL_POINTS);
        for (uint8_t i = 0; i < _cal[s].len; i++) {
            Serial.printf("  %.1fG→%+.4f", _cal[s].table[i].g, _cal[s].table[i].factor);
        }
        Serial.println();
    }
}

// ─── Private ─────────────────────────────────────────────────────────────────

float AccelCalibrationManager::interpolate(const AxisCal& c, float absG) const {
    if (c.len == 0)                  return 0.0f;
    if (absG <= c.table[0].g)        return c.table[0].factor;
    if (absG >= c.table[c.len-1].g)  return c.table[c.len-1].factor;

    for (uint8_t i = 1; i < c.len; i++) {
        if (absG <= c.table[i].g) {
            float span = c.table[i].g - c.table[i-1].g;
            if (span < 1e-6f) return c.table[i].factor;  // guard: duplicate G value
            float t = (absG - c.table[i-1].g) / span;
            return c.table[i-1].factor + t * (c.table[i].factor - c.table[i-1].factor);
        }
    }
    return 0.0f;  // unreachable with correct len/table state
}

void AccelCalibrationManager::sortedInsert(uint8_t sensorIdx, float g, float factor) {
    AxisCal& c = _cal[sensorIdx];

    if (c.len < ACCEL_CAL_POINTS) {
        // Find the correct sorted position.
        uint8_t pos = c.len;
        for (uint8_t i = 0; i < c.len; i++) {
            if (g < c.table[i].g) { pos = i; break; }
        }
        // Shift entries right to open a slot.
        for (uint8_t i = c.len; i > pos; i--) {
            c.table[i] = c.table[i-1];
        }
        c.table[pos] = {g, factor};
        c.len++;
    } else {
        // Table full: overwrite the ring-buffer slot then re-sort in-place.
        // Ring eviction means we cycle through positions 0→7→0, replacing the
        // "oldest" entry rather than the nearest-G entry — same strategy as
        // PotatoMelt.  For 8 entries an insertion sort is negligible.
        c.table[c.evictPos] = {g, factor};
        c.evictPos = (c.evictPos + 1) % ACCEL_CAL_POINTS;

        for (uint8_t i = 1; i < c.len; i++) {
            CalPoint key = c.table[i];
            int8_t   j   = (int8_t)i - 1;
            while (j >= 0 && c.table[j].g > key.g) {
                c.table[j+1] = c.table[j];
                j--;
            }
            c.table[j+1] = key;
        }
    }
}
