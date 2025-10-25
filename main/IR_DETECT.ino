#define IR_REC 1


/*
  angle --> angle of melty
  vel --> velocity of melty
*/


uint16_t angle = 0;
uint16_t vel = 0;


/*
  signal_angle --> Angle of melty where it reads the IR LED
  signal_bounds --> The left and right bounds of where melty should go
*/


uint16_t signal_angle = 0;
uint8_t signal_bounds = 5;




void setup() {
  pinMode(IR_REC, INPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  angle = checkForAngle(angle, vel);
  if (signalFound(angle)) {
    signal_angle = angle;
    correctForSignal(angle, signal_angle);
  }
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


/*
  Checks to see if there is an infrared signal at the angle
*/


bool signalFound() {
  if (digitalRead(IR_REC) == HIGH) {
    return true;
  }
  return false;
}


/*
  Corrects melty movement when moving to infrared signal
*/


void correctForSignal(float angle, float signal_angle) {
  return;
}


// PLACEHOLDER FUNCTION


void getAccel() {
  return;
}




