#include "SensorFusion.h"
#include <math.h>

AccelerometerManager* globalAccel; // optional, for convenience

void SensorFusion::begin(AccelerometerManager* manager) {
    accelManager = manager;
    lastAccelTime = micros();
    lastIRTime    = micros();
}

// -------------------- Non-blocking update --------------------
void SensorFusion::update() {
    unsigned long now = micros();

    // --- Accelerometer ---
    if (now - lastAccelTime >= accelInterval) {
        lastAccelTime = now;
        accelData = accelManager->fetch();
    }

    // --- IR sensor ---
    if (now - lastIRTime >= irInterval) {
        lastIRTime = now;
        readIR();
    }

    // --- Compute orientation ---
    static unsigned long lastFusionTime = 0;
    if (now - lastFusionTime >= accelInterval) { // run fusion at accel rate
        float dt = (now - lastFusionTime) / 1000000.0f;
        lastFusionTime = now;
        computeOrientation(dt);
    }
}

// -------------------- Read IR distance --------------------
void SensorFusion::readIR() {
    // Replace with actual IR sensor reading code
    // For example: analogRead(pin) or a digital LiDAR read
    // Here, we simulate with a placeholder
    irDistance = analogRead(A0) / 1023.0 * 2.0; // simulate 0-2 meters
}

// -------------------- Compute roll/pitch --------------------
void SensorFusion::computeOrientation(float dt) {
    // Compute roll/pitch from accel only (simple complementary filter)
    float alpha = 0.98;

    float accelRoll  = atan2(accelData.y(), accelData.z()) * 180 / PI;
    float accelPitch = atan2(-accelData.x(),
                             sqrt(accelData.y()*accelData.y() + accelData.z()*accelData.z())) * 180 / PI;

    roll  = alpha * roll  + (1.0 - alpha) * accelRoll;
    pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
}

// -------------------- Print --------------------
void SensorFusion::print() {
    unsigned long t = micros();
    Serial.printf("[%010lu us]  Roll: %.2f, Pitch: %.2f, IR: %.2f m\n",
                  t,
                  roll,
                  pitch,
                  irDistance);
}