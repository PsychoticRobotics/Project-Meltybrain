#pragma once

#include <Arduino.h>

// ─── CRSF Protocol Constants ─────────────────────────────────────────────────
// RadioMaster RP2 (ExpressLRS) outputs CRSF at 420000 baud, 8N1.
// Wiring: Receiver TX pad  →  Teensy 4.1 Pin 0  (Serial1 RX)
//         Receiver GND     →  Teensy GND
//         Receiver 5V pad  →  Teensy 5V (or regulated 3.3 V — check your RX)

#define CRSF_NUM_CHANNELS       16      // analog channels in one RC frame
#define CRSF_RAW_MIN            172     // raw 11-bit minimum
#define CRSF_RAW_MID            992     // raw 11-bit midpoint
#define CRSF_RAW_MAX            1811    // raw 11-bit maximum

#define RC_MIN                  1000    // output range in microseconds
#define RC_MID                  1500
#define RC_MAX                  2000

#define CRSF_FAILSAFE_TIMEOUT_MS  500   // ms before link is considered lost

// ─── Status ──────────────────────────────────────────────────────────────────

struct CrsfStatus {
    bool linkOk;        // true: frame received within CRSF_FAILSAFE_TIMEOUT_MS
    uint8_t rssi1;      // RSSI antenna 1 (dBm magnitude, 0 = no data yet)
    uint8_t rssi2;      // RSSI antenna 2
    uint8_t lq;         // Link Quality 0–100 %
    int8_t  snr;        // SNR in dB
    uint8_t rfMode;     // RF mode index reported by TX
    uint8_t txPower;    // TX power index
};

// ─── CrsfReceiver class ───────────────────────────────────────────────────────

class CrsfReceiver {
public:
    /**
     * Set up Serial1 for CRSF.
     * Call once from setup().
     *
     * @param baud  Override baud rate if needed (default 420000).
     */
    void init(uint32_t baud = 420000);

    /**
     * Drain Serial1 and decode any complete CRSF frames.
     * Call every loop() — it is non-blocking.
     *
     * @param channels  Array of exactly CRSF_NUM_CHANNELS uint16_t values.
     *                  Each entry is a channel value in microseconds (RC_MIN–RC_MAX).
     *                  Values are held at their last good reading until a new frame
     *                  arrives, so the array is always safe to read.
     * @param status    Optional pointer to a CrsfStatus struct; filled if provided.
     * @return          true if a new RC-channels frame was decoded this call.
     */
    bool fetch(uint16_t channels[CRSF_NUM_CHANNELS], CrsfStatus* status = nullptr);

    /**
     * Convenience: read a single channel by 1-based index.
     * Returns RC_MID for out-of-range indices.
     */
    uint16_t channel(uint8_t ch) const;

    /** True when no valid frame has arrived within CRSF_FAILSAFE_TIMEOUT_MS. */
    bool isLost() const;

private:
    // Frame parsing
    bool readFrame();
    void decodeRcChannels(const uint8_t* payload);
    void decodeLinkStats(const uint8_t* payload);
    bool crc8DvbS2(const uint8_t* buf, uint8_t len, uint8_t expected) const;

    // Serial buffer
    static constexpr uint8_t CRSF_MAX_FRAME = 64;
    uint8_t _buf[CRSF_MAX_FRAME];
    uint8_t _bufLen  = 0;

    // Channel storage (last-good values, µs)
    uint16_t _channels[CRSF_NUM_CHANNELS] = {};

    // Link stats
    uint8_t _rssi1   = 0;
    uint8_t _rssi2   = 0;
    uint8_t _lq      = 0;
    int8_t  _snr     = 0;
    uint8_t _rfMode  = 0;
    uint8_t _txPower = 0;

    uint32_t _lastFrameMs = 0;

    // Helpers
    static uint16_t mapChannel(uint16_t raw);
};

#endif //MAIN_RECEIVER_H
