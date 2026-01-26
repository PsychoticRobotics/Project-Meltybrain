#include "Accelerometer.h"
#include "Motor.h"
#include "Receiver.h"
#include "Robot.h"
#ifdef B1
#undef B1
#endif
#ifdef B0
#undef B0
#endif
#include "../lib/Eigen/Dense"
#include "Config.h"
#include <Wire.h>


AccelerometerManager accelerometers;
MotorManager motors;
Receiver receiver;
Robot robot(accelerometers, motors);

unsigned previousTime = 0;
unsigned currentTime = 0;

float ch1 = 0;
float ch2 = 0;
float ch3 = 0;

void setup() {
    Serial.begin(19200);
    switch (PROTOCOL) {
        case 0: // SPI
            pinMode(MOSI, OUTPUT);
            pinMode(MISO, INPUT);
            pinMode(SCK, OUTPUT);
            accelerometers.init(10); // Initialize accelerometer with CS pin 10
            break;

        case 1: // I2C
            Wire.begin();
            Wire.setClock(400000); // I2C fast mode
            accelerometers.init(0x18, 0x19); // Initialize accelerometers with I2C addresses 0x18, 0x19
            break;
        default:
            Serial.println("Invalid protocol specified in Config.h");
    }

    receiver.init(2, 3, 4, 5);
    motors.init(13, 15);
    delay(1000); // Motors/ESCs need at least 1 second to arm
}


void loop() {
    ch1 = receiver.fetch(1);
    ch2 = receiver.fetch(2);
    ch3 = receiver.fetch(3);
    //Serial.print("Receiver: ");
    //Serial.print("Ch 1: ");
    //Serial.print(ch1);
    //Serial.print(" Ch 2: ");
    //Serial.print(ch2);
    //Serial.print(" Ch 3: ");
    //Serial.println(ch3);
    currentTime = micros();
    robot.move(ch1, ch2, ch3, currentTime - previousTime);
    previousTime = currentTime;
}