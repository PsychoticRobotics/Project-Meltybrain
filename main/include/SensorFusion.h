//#ifndef SENSORFUSION_H
//#define SENSORFUSION_H
//
//#include "Accelerometer.h"
//#include <Arduino.h>
//
//class SensorFusion {
//public:
//    // Configuration
//    unsigned long accelInterval = 1000; // µs -> 1 kHz
//    unsigned long irInterval    = 20000; // µs -> 50 Hz
//
//    // Latest data
//    Vector3d accelData;
//    float irDistance;  // meters
//    unsigned long lastAccelTime;
//    unsigned long lastIRTime;
//
//    // Orientation (roll/pitch)
//    float roll = 0.0;
//    float pitch = 0.0;
//
//    // References
//    AccelerometerManager* accelManager;
//
//    // Methods
//    void begin(AccelerometerManager* manager);
//    void update();       // non-blocking
//    void print();        // timestamped print
//    void readIR();       // read IR sensor
//    void computeOrientation(float dt);
//};
//
//#endif