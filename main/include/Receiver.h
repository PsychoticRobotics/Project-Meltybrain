#ifndef MAIN_RECEIVER_H
#define MAIN_RECEIVER_H

#include <PulsePosition.h>

class Receiver {
public:
    bool initialized = false;

    Receiver();
    void init(int pin);
    float fetch(int channel);

private:
    PulsePositionInput ReceiverInput{};
};

#endif //MAIN_RECEIVER_H
