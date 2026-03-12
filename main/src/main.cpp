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
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include "../lib/Eigen/Dense"
#include "Config.h"
#include <Wire.h>
#include <SD.h>


AccelerometerManager accelerometers;
MotorManager motors;
Receiver receiver;
Robot robot(accelerometers, motors);

unsigned previousTime = 0;
unsigned currentTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastFlushTime = 0;

float ch1 = 0;
float ch2 = 0;
float ch3 = 0;

const int GREEN_LED_PIN = 6;
const int RED_LED_PIN = 8;

File logger;

void setup() {
    delay(3000);
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
            accelerometers.init(0x18); // Initialize accelerometers with I2C addresses 0x18, 0x19
            break;
        default:
            Serial.println("Invalid protocol specified in Config.h");
    }

        accelerometers.setAdjustments(
        {0.0, -1.0, -1.0}, {1.0, 1.0, 1.0},   // accel1: offset, scale
        {4.229, 4.109, -6.979},   {1.0, 1.0, 1.0}   // accel2: offset, scale
    );

    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);

    if (SD.begin(BUILTIN_SDCARD)) {
        char filename[32];
        int n = 0;
        snprintf(filename, sizeof(filename), "log%d.csv", n);
        while (SD.exists(filename)) {
            n++;
            snprintf(filename, sizeof(filename), "log%d.csv", n);
        }
        logger = SD.open(filename, FILE_WRITE);
        if (logger) {
            logger.println("Time,Acc1_X,Acc1_Y,Acc1_Z,Acc2_X,Acc2_Y,Acc2_Z");
            Serial.print("Logging to: ");
            Serial.println(filename);
        }
    } else {
        Serial.println("SD Card initialization failed");
    }

    receiver.init(2, 3, 4, 5);
    motors.init(13, 15);
    delay(3000); // Motors/ESCs need at least 1 second to arm
}


void loop() {
    // ch1 = receiver.fetch(1);
    // ch2 = receiver.fetch(2);
    // ch3 = receiver.fetch(3);
    // Serial.print("Receiver: ");
    // Serial.print("Ch 1: ");
    // Serial.print(ch1);
    // Serial.print(" Ch 2: ");
    // Serial.print(ch2);
    // Serial.print(" Ch 3: ");
    // Serial.println(ch3);
    currentTime = micros();
    robot.move(0, 0, 0, currentTime - previousTime);

    // Log data at ~200Hz (every 5000 microseconds) to prevent SD card saturation/crashing
    // if (logger && (currentTime - lastLogTime >= 5000)) {
    //     accelerometers.log(logger, currentTime);
    //     lastLogTime = currentTime;
    // }

    // Flush data to disk every 1 second to ensure data is saved if a crash occurs
    // if (logger && (currentTime - lastFlushTime >= 1000000)) {
    //     logger.flush();
    //     lastFlushTime = currentTime;
    // }

    if (robot.theta < PI/8 || robot.theta > 15*PI/8) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, HIGH);
    }
    else {
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, LOW);
    }
    // LED testing: flashes once for 0.02 seconds, looping every 0.1 seconds. This means RPM = 600 if the heading is still.
    if (currentTime % 100000 < 20000) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.println("LED ON");
    }
    else {
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, LOW);
        Serial.println("LED OFF");
    }
    Serial.print("Current time: ");
    Serial.println(currentTime);
    Serial.print("Time since previous: ");
    Serial.println((currentTime - previousTime) / 1000000.0);

    previousTime = currentTime;
}