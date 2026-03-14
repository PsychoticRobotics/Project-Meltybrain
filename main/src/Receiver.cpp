#include "Receiver.h"

// ─── CRSF frame constants ─────────────────────────────────────────────────────

static constexpr uint8_t CRSF_SYNC_BYTE       = 0xC8;  // flight controller address
static constexpr uint8_t CRSF_FRAMETYPE_RC    = 0x16;  // RC channels packed
static constexpr uint8_t CRSF_FRAMETYPE_LINK  = 0x14;  // link statistics
static constexpr uint8_t CRSF_PAYLOAD_RC_LEN  = 22;    // 16 ch × 11 bits = 22 bytes
static constexpr uint8_t CRSF_PAYLOAD_LNK_LEN = 10;

// Frame layout: [sync/addr][length][type][payload...][crc]
// 'length' field = number of bytes from 'type' to end of CRC (inclusive)
// Minimum frame: sync + length + type + crc = 4 bytes

// ─── Public ───────────────────────────────────────────────────────────────────

void CrsfReceiver::init(uint32_t baud) {
    // Pre-fill channels to mid-stick — safe default before first frame.
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        _channels[i] = RC_MID;
    }

    // Serial1: Pin 0 (RX1), Pin 1 (TX1) on Teensy 4.1
    // CRSF is standard non-inverted 8N1.
    Serial1.begin(baud, SERIAL_8N1);
}

bool CrsfReceiver::fetch(uint16_t channels[CRSF_NUM_CHANNELS], CrsfStatus* status) {
    bool gotRcFrame = false;

    // Read all available bytes and try to assemble complete frames.
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();

        // If buffer is empty, only accept a sync byte to start.
        if (_bufLen == 0) {
            if (b != CRSF_SYNC_BYTE) continue;
        }

        // Guard against overrun (malformed stream).
        if (_bufLen >= CRSF_MAX_FRAME) {
            _bufLen = 0;
            continue;
        }

        _buf[_bufLen++] = b;

        // We need at least 2 bytes (sync + length) to know the frame size.
        if (_bufLen < 2) continue;

        uint8_t frameLen = _buf[1];          // bytes remaining after the length field
        uint8_t totalLen = 2 + frameLen;     // sync + length + type + payload + crc

        // Sanity-check the declared length.
        if (frameLen < 2 || totalLen > CRSF_MAX_FRAME) {
            _bufLen = 0;   // discard and resync
            continue;
        }

        // Not enough data yet — keep buffering.
        if (_bufLen < totalLen) continue;

        // We have a complete frame — validate CRC.
        // CRC covers bytes from 'type' byte to end of payload (excludes sync, length, crc).
        uint8_t crcPos = totalLen - 1;
        if (crc8DvbS2(&_buf[2], frameLen - 1, _buf[crcPos])) {
            uint8_t frameType = _buf[2];
            const uint8_t* payload = &_buf[3];

            if (frameType == CRSF_FRAMETYPE_RC && frameLen - 2 == CRSF_PAYLOAD_RC_LEN) {
                decodeRcChannels(payload);
                _lastFrameMs = millis();
                gotRcFrame = true;
            } else if (frameType == CRSF_FRAMETYPE_LINK && frameLen - 2 == CRSF_PAYLOAD_LNK_LEN) {
                decodeLinkStats(payload);
            }
        }

        // Consume the frame from the buffer regardless of CRC result.
        _bufLen = 0;
    }

    // Copy last-good channel values into the caller's array.
    memcpy(channels, _channels, sizeof(uint16_t) * CRSF_NUM_CHANNELS);

    if (status) {
        status->linkOk  = !isLost();
        status->rssi1   = _rssi1;
        status->rssi2   = _rssi2;
        status->lq      = _lq;
        status->snr     = _snr;
        status->rfMode  = _rfMode;
        status->txPower = _txPower;
    }

    return gotRcFrame;
}

uint16_t CrsfReceiver::channel(uint8_t ch) const {
    if (ch < 1 || ch > CRSF_NUM_CHANNELS) return RC_MID;
    return _channels[ch - 1];
}

bool CrsfReceiver::isLost() const {
    return (_lastFrameMs == 0) ||
           ((millis() - _lastFrameMs) >= CRSF_FAILSAFE_TIMEOUT_MS);
}

// ─── Private ─────────────────────────────────────────────────────────────────

/**
 * Decode a 22-byte CRSF RC-channels payload.
 * 16 channels packed as 11-bit values, LSB first.
 */
void CrsfReceiver::decodeRcChannels(const uint8_t* p) {
    uint16_t raw[CRSF_NUM_CHANNELS];

    raw[0]  = ((p[0]        | (uint16_t)p[1]  << 8)  & 0x07FF);
    raw[1]  = ((p[1]  >> 3  | (uint16_t)p[2]  << 5)  & 0x07FF);
    raw[2]  = ((p[2]  >> 6  | (uint16_t)p[3]  << 2  | (uint16_t)p[4]  << 10) & 0x07FF);
    raw[3]  = ((p[4]  >> 1  | (uint16_t)p[5]  << 7)  & 0x07FF);
    raw[4]  = ((p[5]  >> 4  | (uint16_t)p[6]  << 4)  & 0x07FF);
    raw[5]  = ((p[6]  >> 7  | (uint16_t)p[7]  << 1  | (uint16_t)p[8]  << 9)  & 0x07FF);
    raw[6]  = ((p[8]  >> 2  | (uint16_t)p[9]  << 6)  & 0x07FF);
    raw[7]  = ((p[9]  >> 5  | (uint16_t)p[10] << 3)  & 0x07FF);
    raw[8]  = ((p[11]       | (uint16_t)p[12] << 8)  & 0x07FF);
    raw[9]  = ((p[12] >> 3  | (uint16_t)p[13] << 5)  & 0x07FF);
    raw[10] = ((p[13] >> 6  | (uint16_t)p[14] << 2  | (uint16_t)p[15] << 10) & 0x07FF);
    raw[11] = ((p[15] >> 1  | (uint16_t)p[16] << 7)  & 0x07FF);
    raw[12] = ((p[16] >> 4  | (uint16_t)p[17] << 4)  & 0x07FF);
    raw[13] = ((p[17] >> 7  | (uint16_t)p[18] << 1  | (uint16_t)p[19] << 9)  & 0x07FF);
    raw[14] = ((p[19] >> 2  | (uint16_t)p[20] << 6)  & 0x07FF);
    raw[15] = ((p[20] >> 5  | (uint16_t)p[21] << 3)  & 0x07FF);

    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        _channels[i] = mapChannel(raw[i]);
    }
}

/**
 * Decode a 10-byte CRSF link-statistics payload.
 * Byte order per the CRSF spec:
 *   [0] uplink RSSI ant1  [1] uplink RSSI ant2  [2] uplink LQ
 *   [3] uplink SNR        [4] active antenna    [5] RF mode
 *   [6] TX power          [7] downlink RSSI     [8] downlink LQ
 *   [9] downlink SNR
 */
void CrsfReceiver::decodeLinkStats(const uint8_t* p) {
    _rssi1    = p[0];
    _rssi2    = p[1];
    _lq       = p[2];
    _snr      = (int8_t)p[3];
    _rfMode   = p[5];
    _txPower  = p[6];
}

/**
 * CRC-8/DVB-S2: polynomial 0xD5.
 * Covers the type byte and payload; excludes sync, length, and the CRC byte itself.
 */
bool CrsfReceiver::crc8DvbS2(const uint8_t* buf, uint8_t len, uint8_t expected) const {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc == expected;
}

/**
 * Map raw 11-bit CRSF value (172–1811) to microseconds (1000–2000).
 */
uint16_t CrsfReceiver::mapChannel(uint16_t raw) {
    uint16_t us = (uint16_t)map((long)raw, CRSF_RAW_MIN, CRSF_RAW_MAX, RC_MIN, RC_MAX);
    return constrain(us, RC_MIN, RC_MAX);
}