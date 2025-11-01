#include <Arduino.h>

#define IR_REC 1

class IR {
public:
    // angle --> angle of melty
    // vel --> velocity of melty
    uint16_t angle = 0;
    uint16_t vel = 0;
    // signal_angle --> Angle of melty where it reads the IR LED
    // signal_bounds --> The left and right bounds of where melty should go
    uint16_t signal_angle = 0;
    uint8_t signal_bounds = 5;
    // IRinitialize --> Initializes IR Reciever
    void IRinitialize()
    // getAngle --> Determines angle of melty relative to angle at startup
    uint16_t getAngle(float angle, uint64_t vel)
    // signalFound --> Checks whether there is a signal at current angle
    bool signalFound()
    // correctForSignal --> Moves melty in direction of signal angle
    void correctForSignal(float angle, float signal_angle)
}