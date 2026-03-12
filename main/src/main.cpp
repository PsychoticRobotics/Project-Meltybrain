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
    Serial.begin(9600);
    delay(1000);
    if (CrashReport) {
        Serial.print(CrashReport); // Print any previous crash info
    }
    Serial.println("--- SETUP START ---");

    Serial.println("Initializing Accelerometers...");
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
            Serial.println("FATAL: Invalid protocol specified in Config.h. Halting.");
            while(1); // Halt execution
    }
    Serial.println("...Accelerometers Initialized.");

    Serial.println("Initializing Receiver...");
    receiver.init(2, 3, 4, 5);
    Serial.println("...Receiver Initialized.");

    Serial.println("Initializing Motors & Arming ESCs...");
    motors.init(13, 15);
    delay(1000); // Motors/ESCs need at least 1 second to arm
    Serial.println("...Motors Armed.");

    Serial.println("--- SETUP COMPLETE, entering main loop ---");
    previousTime = micros();
}


void loop() {
    ch1 = receiver.fetch(1);
    ch2 = receiver.fetch(2);
    ch3 = receiver.fetch(3);
    // Serial.print("Receiver: ");
    // Serial.print("Ch 1: ");
    // Serial.print(ch1);
    // Serial.print(" Ch 2: ");
    // Serial.print(ch2);
    // Serial.print(" Ch 3: ");
    // Serial.println(ch3);
    currentTime = micros();
    robot.move(ch1, ch2, ch3, currentTime - previousTime);
    previousTime = currentTime;
    delay(20); // Small delay to make serial output readable and prevent buffer overflow
}