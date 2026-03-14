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
CrsfReceiver rc;
Robot robot(accelerometers, motors);

unsigned previousTime = 0;
unsigned currentTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastFlushTime = 0;

uint16_t channels[CRSF_NUM_CHANNELS];
CrsfStatus status;

const int GREEN_LED_PIN = 6;
const int RED_LED_PIN = 8;

File logger;

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
            accelerometers.init(0x18); // Initialize accelerometers with I2C addresses 0x18, 0x19
            break;
        default:
            Serial.println("FATAL: Invalid protocol specified in Config.h. Halting.");
            while(1); // Halt execution
    }
    Serial.println("...Accelerometers Initialized.");

    Serial.println("Initializing Receiver...");
    rc.init();
    Serial.println("...Receiver Initialized.");

    Serial.println("Initializing Motors & Arming ESCs...");
    motors.init(13, 15);
    delay(1000); // Motors/ESCs need at least 1 second to arm
    Serial.println("...Motors Armed.");

    Serial.println("--- SETUP COMPLETE, entering main loop ---");
    previousTime = micros();
}


void loop() {
    rc.fetch(channels, &status);
    if (rc.isLost()) {
        Serial.println("CRSF signal lost! Halting.");
        while (1) {
            robot.move(0, 0, 0, 0);
        }
    }
     Serial.print("Receiver: ");
     Serial.print("Ch 1: ");
     Serial.print(channels[0]);
     Serial.print(" Ch 2: ");
     Serial.print(channels[1]);
     Serial.print(" Ch 3: ");
     Serial.println(channels[2]);
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
    Serial.println((currentTime - previousTime) / 1000000.0, 6);

    previousTime = currentTime;
}