#include "Accelerometer.h"
#include "Motor.h"
#include "Receiver.h"
#include "Robot.h"
#include "../lib/Eigen/Dense"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>


AccelerometerManager accelerometers;
MotorManager motors;
Receiver receiver;
Robot robot(accelerometers, motors);

unsigned previousTime = 0;
unsigned currentTime = 0;

[[noreturn]] void quit() {
    Serial.println("Exiting program.");
    while (true) {
        // Infinite loop to halt execution
    }
}

void setup() {
    Serial.begin(19200);
    switch (PROTOCOL) {
        case 0: // SPI
            pinMode(MOSI, OUTPUT);
            pinMode(MISO, INPUT);
            pinMode(SCK, OUTPUT);

            if (!accelerometers.init(10)) quit(); // Initialize accelerometer with CS pin 10

            break;
        case 1: // I2C
            Wire.begin();
            Wire.setClock(400000); // I2C fast mode

            if (!accelerometers.init(0x19, 0x18)) quit(); // Initialize accelerometers with I2C addresses 0x18, 0x19

            break;
        default:
            Serial.println("Invalid protocol specified in Config.h");
            quit();
    }

    receiver.init(2); // Initialize receiver on pin 2
    motors.init(14, 15);
    delay(1000); // Motors/ESCs need at least 1 second to arm
}

void loop() {
    Serial.print("Receiver: ");
    Serial.print("Ch 1: ");
    Serial.print(receiver.fetch(1));
    Serial.print(" Ch 2: ");
    Serial.print(receiver.fetch(2));
    Serial.print(" Ch 3: ");
    Serial.println(receiver.fetch(3));
    Serial.print("Accelerometer NTU: ");
    Vector3d accelData = accelerometers.fetchNTU();
    Serial.print("N: ");
    Serial.print(accelData.x());
    Serial.print(" T: ");
    Serial.print(accelData.y());
    Serial.print(" U: ");
    Serial.println(accelData.z());
    delay(200);
    // currentTime = micros();
    // robot.move(0.0f, 0.0f, 0.0f, currentTime - previousTime);
    // previousTime = currentTime;
}