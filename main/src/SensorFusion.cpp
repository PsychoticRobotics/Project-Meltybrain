#include "SensorFusion.h"
#include <math.h>

// This file should read the status of the sensors
// Then it should output the estimated orientation

void SensorFusion::begin() {
    lastAccelTime = micros();
    lastIRTime    = micros();
    lastMagTime   = micros();
}

// ----------- Non-blocking update (updates what is available) -----------
void SensorFusion::update() {
    unsigned long now = micros();

    // --- Accelerometer ---
    if (now - lastAccelTime >= accelInterval) {
        lastAccelTime = now;
        // readAccel();
    }

    // --- IR sensor ---
    if (now - lastIRTime >= irInterval) {
        lastIRTime = now;
        // readIR();
    }

    // --- Magnetometer ---
    if (now - lastMagTime >= magnInterval) {
        lastMagTime = now;
        // readMag();
    }

    // --- Compute orientation ---
    static unsigned long lastFusionTime = 0;
    if (now - lastFusionTime >= accelInterval) { // run fusion at accel rate
        float dt = (now - lastFusionTime) / 1000000.0f;
        lastFusionTime = now;
        computeOrientation(dt);
    }
}

// Accelerometer Filter

// IR Filter

// Magnetometer Filter

// -------------------- Compute angle (theta) --------------------
void SensorFusion::computeTheta() { // originally had float dt as input
    /* Will write soon */

}

// -------------------- Print --------------------
void SensorFusion::print() {
    unsigned long t = micros();
    Serial.printf("[%010lu us] Orientation: %d", orientation);
}