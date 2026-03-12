#ifndef MAIN_RECEIVER_H
#define MAIN_RECEIVER_H

#include <PulsePosition.h>

class Receiver {
public:
    bool initialized = false;

    void init(int pin1, int pin2, int pin3, int pin4);
    float fetch(int channel);

private:
    int pin1, pin2, pin3, pin4;
};

#endif //MAIN_RECEIVER_H
