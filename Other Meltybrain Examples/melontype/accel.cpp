#include "config.h"
#include "Adafruit_H3LIS331.h"
#include "accel.h"
#include "LED.h"

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

volatile float accelAngle = 0;
volatile float accelMag = 0;
volatile float angularPosition = 0;  // phase was a bad global variable name.
// angularPosition this is the current position within a single rotation, represented as a number between 0 and 1.
// The initial direction is effectively randomly chosen. you can adjust the offset with the right potentiometer on the transmitter (I'm assuming you have the FS-i6x or the FS-i6 with the 10 channel firmware patch)

IntervalTimer accTimer;
sensors_event_t sensor;

extern void checkSerial();
extern void updateInputs();
extern void displayState();

void accelEvent();

void initAccel() {
  setCode(0011); 
  // remove attempt to power I2C device from GPIO: powered from 3v3 directly now
  // - higher current available and it's what it's for

  if (!lis.begin_I2C(ACCEL_I2C_ADDR)) {
    Serial.println("Failed to initialize accelerometer!");
    Serial.println("Use serial commands now if you want. I don't care.");
    Serial.flush();
    while (1) {
      checkSerial();  // for command parsing and stuff.
      updateInputs();  // update the inputs
      displayState();      
      //updateLEDs(); 
      setCode(0203); // 2 red, 0 green, 3 blue
      delay(50);  // 
      setCode( 0230); // 2 red, 3 green, 0 blue
      delay(50);  
    }
    // We stop the system if the accelerometer initialization fails.
    // it's kind of important.
  }

  // Configure the accelerometer settings
  lis.setDataRate(LIS331_DATARATE_400_HZ); 
  lis.setRange(H3LIS331_RANGE_400_G); // at 54mm radius 400G equals 2574 RPM
      
  accTimer.begin(accelEvent, 1e6 / 400);

  Serial.println("Accelerometer initialized successfully.");
  setCode(0020);
}

float filterAlpha = 0.05;
float previousFilteredReading = 0.0;

// Apply a low-pass filter to the accelerometer reading
float lowPassFilter(float currentReading) {
  previousFilteredReading = filterAlpha * currentReading + (1.0 - filterAlpha) * previousFilteredReading;
  return previousFilteredReading;
}

void accelEvent()
{
  lis.getEvent(&sensor);
  // Subtract calibration offsets -- print out the values with the accelerometer oriented in every angle and work out the ranges and centers. 
  // (put it 0/180 degrees on each axis for a minute or two while logging the values to calculate the offsets, then put them in accelOffset in config.h)
  float x = sensor.acceleration.x -= accelOffsetX;
  float y = sensor.acceleration.y -= accelOffsetY;
  float z = sensor.acceleration.z -= accelOffsetZ;
  (void)(z);
  accelAngle = atan2(y, x) / M_TWOPI;
  // let's make the global accelMag setting slightly closer to atomic
  // otherwise it might get referenced between lines...
  float tmpAccelMag   = abs(z); // Neether uses Z. // abs(x); //  sqrt(x * x + y * y);
  accelMag = lowPassFilter(tmpAccelMag);
  if(accelMag < min_rotation_G)
  {
    accelMag = min_rotation_G;
    setRGB(0xff, 0x00, 0x00, 11);
  }
  else
  {
    setRGB(0x00, 0xff, 0x00, 11);
  }
} 
