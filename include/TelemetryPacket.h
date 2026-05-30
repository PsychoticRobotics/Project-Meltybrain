#ifndef MELTYBRAIN_TELEMETRY_PACKET_H
#define MELTYBRAIN_TELEMETRY_PACKET_H

// ─── Shared telemetry packet definition ────────────────────────────────────────
// Copy this file VERBATIM into the ESP32 receiver project so both ends agree on
// layout, framing, baud, and checksum. It is the single source of truth for the
// wire format — change it in one place and update the copy on the other side.
//
// Wire format on the UART (one frame):
//
//   [SYNC0][SYNC1][ TelemetryPacket bytes ... ][CRC8]
//
//   • SYNC0/SYNC1  let the receiver re-find a frame boundary after a dropped byte
//   • CRC8         is computed over the TelemetryPacket bytes only (not the syncs)
//
// Both Teensy 4.x (Cortex-M7) and the ESP32 (Xtensa/RISC-V) are 32-bit and
// little-endian, so a raw byte copy of the packed struct is portable between
// them. The struct is byte-packed so neither compiler inserts padding.

#include <stdint.h>

#define TELEM_SYNC0        0xAA
#define TELEM_SYNC1        0x55
#define TELEM_BAUD         921600   // Serial4 TX (pin 17) → ESP32 RX. Lower both
                                    // ends together if long wires prove flaky.
#define TELEM_INTERVAL_US  10000    // send rate limit: 10 ms → 100 Hz

#pragma pack(push, 1)
struct TelemetryPacket {
    uint32_t timestamp_us;   // Teensy micros() at send time
    float    angle;          // fused heading, radians [0, 2π]
    float    omega;          // fused spin rate, rad/s
    float    magRPM;         // magnetometer-derived spin rate, RPM (body spin)
    int16_t  rpm1;           // ESC1 electrical RPM
    int16_t  rpm2;           // ESC2 electrical RPM
    float    volts1;         // ESC1 supply voltage, V
    float    volts2;         // ESC2 supply voltage, V
    float    amps1;          // ESC1 current, A
    float    amps2;          // ESC2 current, A
    uint8_t  temp1;          // ESC1 temperature, °C
    uint8_t  temp2;          // ESC2 temperature, °C
    uint16_t ch1;            // RC channel 1 (raw µs)
    uint16_t ch2;            // RC channel 2 (raw µs)
    uint16_t ch3;            // RC channel 3 (raw µs)
    uint8_t  flags;          // see TELEM_FLAG_* below
    float    pos_x;          // arena position, metres from centre (+X = right)
    float    pos_y;          // arena position, metres from centre (+Y = forward)
                             // only valid when TELEM_FLAG_POS_VALID is set
    float    opp_bearing;    // bearing (radians, 0–2π) to nearest opponent target
                             // only valid when TELEM_FLAG_OPP_VALID is set
};
#pragma pack(pop)

// flags bit field
#define TELEM_FLAG_RC_LOST    0x01   // receiver signal lost
#define TELEM_FLAG_MAG_VALID  0x02   // magnetometer angle fix is valid
#define TELEM_FLAG_SPINNING   0x04   // spin rate estimate is valid
#define TELEM_FLAG_POS_VALID  0x08   // pos_x / pos_y are valid (IR arena tracker)
#define TELEM_FLAG_OPP_VALID  0x10   // opp_bearing is valid (opponent detected)

// CRC-8 (poly 0x07, init 0x00). Defined inline so both ends share identical code.
static inline uint8_t telem_crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

#endif // MELTYBRAIN_TELEMETRY_PACKET_H
