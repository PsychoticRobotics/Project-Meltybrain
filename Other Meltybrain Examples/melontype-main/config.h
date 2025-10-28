#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <Arduino.h>

// The 9 pins on the right side of a Teensy 4.x are:
// 
// Vin   - BEC to Teensy,  5v of receiver and 5v of LED strip.
// GND   - connects to common ground. [shared with accelerometer, reciever, Teensy, BEC, and two ESCs]
// 3.3v  - supplies ~300mA current at 3v3. powers accelerometer.
// 
// 23 - controls motor_2 ESC
// 22 - controls motor_1 ESC
// 21 - RX5  listening to the IBus receiver output
// 20 - controls signal pin of the NeoPixels
// 19 - SCL0 - SCL for accelerometer
// 18 - SDA0 - SDA for accelerometer
// 



const int accelSDA    = 18;
const int accelSCL    = 19;
#define ACCEL_I2C_ADDR 0x18 // default for adafruit h3lis331 board - works with 3v3 and 5v MCUs
//#define ACCEL_I2C_ADDR 0x19   // default for sparkfun h3lis331 board - works with 3v3 MCUs.
const int LEDpin    = 20;    // TX5 isn't used. LEDpin uses this one. (optimizing pins to be physically close)
const int altTXpin  = 39;    // this is to protect pin 20 from Serial5 grabbing it.
const int ibusPin   = 21;    // RX5 pin. Hardware5 is digital 20/21  if changed, update in melontype.ino too!
const int motorPin2 = 22;    // 22 next to top pin - match pin to PCB
const int motorPin1 = 23;    // 23 topmost pin     - match pin to PCB

const int ESC_SERVO_RATE = 400;    // Hz
const float ZERO_THRESHOLD = 0.1;  // throttle fractionm so ZERO_THRESHOLD * 

const int PWM_1000us = 65536 * 40 / 100;  
const int PWM_2000us = 65536 * 80 / 100;  

const float boostThreshold = 0.2;        
const float boostSpeed = 0.5;            
const unsigned long baseCycleTime = 50;  
const unsigned long coolDownTime = 100;

const float min_rotation_G = 0.3; 

// empirically dereived by watching the numbers on my accelerometer.
// basically took the largest value seen while each of the X, -X, Y, -Y, Z, or -Z orientations is up, and then average the pairs.
// these offsets, when added to the data should move x,y, and z all back into the range -10..10 when you have that axis up/down.
// i.e. this should approximate the average error for each axis.
const float accelOffset[3] = {
   -25.86, -22.98, 47.88       // chip3 -- Neether current
  // -0.48 , -3.83,  -5.75     // chip1 (must *9.81 m/s^2)
  //  -0.75, -0.48,  5.75      // chip2 (    "       "   )
  //   0.00,  0.00,  0.00      // zeros..
};

const float accelOffsetX = accelOffset[0];
const float accelOffsetY = accelOffset[1];
const float accelOffsetZ = accelOffset[2];

#endif