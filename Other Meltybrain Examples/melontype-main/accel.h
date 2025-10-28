#ifndef __accel_h__
#define __accel_h__

extern volatile float accelAngle;
extern volatile float accelMag;

extern volatile float angularPosition;  // phase was a bad global variable name, and this is slightly better. slightly.

// angularPosition this is the current position within a single rotation, represented as a number between 0 and 1.
// The initial direction is effectively randomly chosen. you can adjust the offset with the right potentiometer on 
// the transmitter (I'm assuming you have the FS-i6x or the FS-i6 with the 10 channel firmware patch)

void initAccel();
void collectCalibrationData();

#endif // __accel_h__