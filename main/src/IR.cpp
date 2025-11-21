#include <Arduino.h>
#include "IR.h"

void IRinitialize() {
  pinMode(IR_REC, INPUT)
}

uint16_t checkForAngle(float angle, uint64_t vel) {
  /*
    interval --> Interval of microseconds
    elapsed_micros --> Amount of microseconds elapsed since BEGINNING OF RUNTIME
    previous_micros --> Amount of microseconds elapsed since LAST COMPLETED INTERVAL
  */

  const uint8_t interval = 100;
  uint32_t elapsed_micros = micros();
  uint8_t previous_micros = 0;

  /*
    current_accel --> Acceleration of melty at the end of the interval
  */

  uint16_t current_accel;

  if (elapsed_micros - previous_micros > interval) {
    // Gets acceleration post interval
    current_accel = getAccel();
    angle += angle + vel * interval + 0.5 * current_accel * pow(interval, 2);
  }
  
  return angle;
}

bool signalFound() {
  if (digitalRead(IR_REC) == HIGH) {
    return true;
  }
  return false;
}


/*
  **WIP** Moves melty in direction of IR
  {This would go in the main.cpp file. Can be written here for now, but eventually
  it will go in main. - Peter}
*/


void correctForSignal(float angle, float signal_angle) {
  return;
}



