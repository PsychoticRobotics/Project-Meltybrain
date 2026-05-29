#ifndef MELTYBRAIN_TELEMETRY_H
#define MELTYBRAIN_TELEMETRY_H

// Usage:
//
//   Telemetry telemetry(estimator, mag, motors);
//
//   setup():
//     telemetry.init();
//
//   loop():
//     telemetry.update(currentTime, channels, rc.isLost());
//
// Streams a binary TelemetryPacket out Serial4 (TX pin 17) to an ESP32,
// rate-limited internally to TELEM_INTERVAL_US. See TelemetryPacket.h for the
// wire format. One-way for now; Serial4 RX (pin 16) is left free so this can be
// extended to bidirectional later without moving pins.

#include <stdint.h>
#include "TelemetryPacket.h"

// Forward declarations keep this header light — full definitions are only
// needed in Telemetry.cpp.
class AngleEstimator;
class MagnetometerTracker;
class MotorManager;

class Telemetry {
public:
    Telemetry(AngleEstimator& estimator,
              MagnetometerTracker& mag,
              MotorManager& motors);

    void init();                                              // opens Serial4
    void update(uint32_t t_us,
                const uint16_t* channels,                     // RC channels (may be null)
                bool rcLost);                                 // builds + sends if due

private:
    void send(const TelemetryPacket& pkt);                    // frame + write

    AngleEstimator*      _estimator;
    MagnetometerTracker* _mag;
    MotorManager*        _motors;
    uint32_t             _lastSend = 0;
};

#endif // MELTYBRAIN_TELEMETRY_H
