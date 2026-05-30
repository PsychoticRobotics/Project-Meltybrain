#include "Telemetry.h"
#include "AngleEstimator.h"
#include "Magnetometer.h"
#include "Motor.h"
#include <Arduino.h>
#include <string.h>

Telemetry::Telemetry(AngleEstimator& estimator,
                     MagnetometerTracker& mag,
                     MotorManager& motors)
    : _estimator(&estimator), _mag(&mag), _motors(&motors) {}

void Telemetry::init() {
    Serial4.begin(TELEM_BAUD);
}

void Telemetry::update(uint32_t t_us, const uint16_t* channels, bool rcLost) {
    // Rate limit. Unsigned subtraction handles micros() wraparound correctly.
    if (t_us - _lastSend < TELEM_INTERVAL_US) return;
    _lastSend = t_us;

    TelemetryPacket pkt = {};            // zero-init: missing fields stay 0
    pkt.timestamp_us = t_us;
    pkt.angle  = _estimator->getAngle();
    pkt.omega  = _estimator->getOmega();
    pkt.magRPM = _mag->getRPM();

    // ESC telemetry — getters return false when the data isn't valid yet, in
    // which case the corresponding field keeps its zero-initialised value.
    int16_t rpm   = 0;
    float   value = 0.0f;
    uint8_t deg   = 0;
    if (_motors->getRPM(1, rpm))       pkt.rpm1   = rpm;
    if (_motors->getRPM(2, rpm))       pkt.rpm2   = rpm;
    if (_motors->getVoltage(1, value)) pkt.volts1 = value;
    if (_motors->getVoltage(2, value)) pkt.volts2 = value;
    if (_motors->getCurrent(1, value)) pkt.amps1  = value;
    if (_motors->getCurrent(2, value)) pkt.amps2  = value;
    if (_motors->getTemp(1, deg))      pkt.temp1  = deg;
    if (_motors->getTemp(2, deg))      pkt.temp2  = deg;

    if (channels) {
        pkt.ch1 = channels[0];
        pkt.ch2 = channels[1];
        pkt.ch3 = channels[2];
    }

    pkt.flags = 0;
    if (rcLost)               pkt.flags |= TELEM_FLAG_RC_LOST;
    if (_mag->isAngleValid()) pkt.flags |= TELEM_FLAG_MAG_VALID;
    if (_mag->isSpinning())   pkt.flags |= TELEM_FLAG_SPINNING;
    if (_posValid) {
        pkt.flags |= TELEM_FLAG_POS_VALID;
        pkt.pos_x  = _posX;
        pkt.pos_y  = _posY;
    }

    send(pkt);
}

void Telemetry::setPosition(float x, float y) {
    _posX     = x;
    _posY     = y;
    _posValid = true;
}

void Telemetry::clearPosition() {
    _posX     = 0.0f;
    _posY     = 0.0f;
    _posValid = false;
}

void Telemetry::send(const TelemetryPacket& pkt) {
    // Assemble the whole frame in one buffer, then write once: SYNC0, SYNC1,
    // payload, CRC8 (over payload only).
    uint8_t frame[3 + sizeof(TelemetryPacket)];
    frame[0] = TELEM_SYNC0;
    frame[1] = TELEM_SYNC1;
    memcpy(&frame[2], &pkt, sizeof(pkt));
    frame[2 + sizeof(pkt)] = telem_crc8(frame + 2, sizeof(pkt));

    Serial4.write(frame, sizeof(frame));
}
