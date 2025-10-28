// this is terrible code. not even being self-deprecating. so use this at your own risk.
// read the line above this again, and if you keep going, remember that it's all your fault now.

// previous header comment/documentation is now in "original_header.h"

#include <IBusBM.h>
#include "config.h"
#include "LED.h"
#include "accel.h"
#include "font.h"
#include "Adafruit_Sensor.h"
#include "dynamic_calibration.h"

// Declare objects and variables
IBusBM ibus;  // IBus object for the radio receiver

void pwmInterruptHandler();

// Update PID constants struct to support bidirectional range
struct PIDConstants {
  // prep for 200.0f
  float kP = 50.0f / 3800.0f;      // Proportional gain 
  float kI =  0.0f; // 0.0001f;     // Integral gain
  float kD =  0.0f; // 0.001f;      // Derivative gain
  float MAX_RPM = 3800.0f; // Hard cap on RPM - more reasonable than 4.5k
  float OUTPUT_CAP = 1.0f; // Maximum throttle output magnitude
  float MIN_THROTTLE = 0.05f; // Minimum throttle magnitude to overcome friction
  float MAX_INTEGRAL = 100.0f; // Anti-windup limit
};

// PID controller state
struct PIDState {
  float targetRPM = 0.0f;
  float lastError = 0.0f;
  float errorIntegral = 0.0f;
  unsigned long lastPIDUpdate = 0;
};

// Phase tracking state
struct PhaseState {
  float continuousPhase = 0.0f;
  unsigned long lastPhaseUpdate = 0;
  unsigned long usRevStartTime = 0;
};

// Throttle values for both motors
struct ThrottleValues {
  float th1;
  float th2;
};

// Global PID constants and state
PIDConstants pidConstants;
PIDState pidStateM1;
PIDState pidStateM2;
PhaseState phaseState;


//HardwareSerial &iBusSerialPort = Serial2;  // digital pin 7: update ibusPin in config.h too!!!!
HardwareSerialIMXRT &iBusSerialPort = Serial5;  // digital pin 21: update ibusPin in config.h ::::: new board only!!
IntervalTimer pwmTimer;

bool pwmIntCalled;
bool doResetStats = false;

volatile uint32_t last_ibus_seen_millis;
volatile uint8_t last_cnt_rec;
volatile uint8_t last_cnt_poll;

// Global variables for motor throttles
float motor1Throttle = 0.0;
float motor2Throttle = 0.0;
int lastMotor1 = -1;  // make them different to get the update.
int lastMotor2 = -1;  // ditto. (renamed, sorry.)


// Variables to store inputs
volatile float stickVert = 0.0;      // up/down = ch0
volatile float stickHoriz = 0.0;     // left/right = ch1
volatile float stickAngle = 0.0;     // Stick angle as a fraction of a circle
volatile float stickLength = 0.0;    // Length of the stick vector
volatile float powerInput = 0.0;       // Throttle input = ch2
volatile float sidewaysInput = 0.0;  // Sideways input = ch3
volatile float radiusInput = 0.0;    // Radius input = ch4
volatile float radiusSize = 0.01;    // Mapped radius size (0.1 cm to 5 cm) in meters
volatile float headingOffset = 0.0;  // offset around circle heading light should be = ch5
volatile bool reverseM1 = false;
volatile bool reverseM2 = false;
volatile int sw3pos = 0;
volatile int swReverseText = false;

volatile float g_cos_ph1 = 0;

// map the throttle input so it spends more time in the low areas.
float stretchThrottle(float throttle) {
  float s = throttle > 0 ? 1.f : -1.f;
  return s * (throttle * throttle);
}

int throttleToPWM(float input, bool stretch) {
  // Constrain the throttle value between 0 and 1
  input = constrain(input, -1.0, 1.0);
  // do non-linear stretch or not?
  if (stretch) {
    input = stretchThrottle(input);
  }
  // Map the throttle value (-1 to 1) to the PWM duty cycle values (1000us and 2000us converted to the 400Hz PWM)
  return map(input, -1, 1, PWM_1000us, PWM_2000us);
}

// Modified setThrottle to only store the desired throttle, no analogWrite() here
void setThrottle(float throttle1, float throttle2, bool stretch = false) {
  if (throttle1 == 0 && throttle2 == 0) {
    lastMotor1 = -2;  // -2 is impossible to actually set
    lastMotor2 = -2;  // force an update. hack. so? shut up.
  }

  if (reverseM1) throttle1 = -throttle1;
  if (reverseM2) throttle2 = -throttle2;

  motor1Throttle = throttleToPWM(throttle1, stretch);
  motor2Throttle = throttleToPWM(throttle2, stretch);
  pwmIntCalled = false;
}


unsigned long endBoost[2] = { 0, 0 };
bool isBoost[2] = { true, true };  // Flag to track whether we're sending modSpeed or 0

// Function to modulate throttle for low-speed control
// this is intended to give a small boost when starting up if the throttle is below 0.2
// to make it easier to keep moving at low speed.
float modulateThrottle(float inputThrottle, int idx) {
  unsigned long now = millis();
  if (now > endBoost[idx] + coolDownTime) {
    endBoost[idx] = 0;
    return inputThrottle;
  }
  if (now > endBoost[idx] || inputThrottle > boostThreshold) {
    isBoost[idx] = false;
    return inputThrottle;
  }
  if (endBoost[idx] == 0) {
    isBoost[idx] = true;
    endBoost[idx] = now + baseCycleTime;
  }
  if (!isBoost[idx]) return inputThrottle;
  return boostSpeed;
}


// Function to handle tank drive logic
// convert from 2-axis stick input into two channel "tank drive" motor output.
// (converts stick to "tank mode" output)
void handleTankDrive() {
  float x = stickHoriz / 1.0;  // Horizontal stick input -1.0 - 1.0
  float y = stickVert / 1.0;   // Vertical stick input   -1.0 - 1.0

  // Calculate motor control values based on stick input
  float m1 = x + y;  // Motor 1 throttle
  float m2 = x - y;  // Motor 2 throttle

  // this one is for trying to stop it taking off so fast in tank mode.
  m1 = modulateThrottle(m1, 0);
  m2 = modulateThrottle(m2, 1);

  pwmIntCalled = false;
  setThrottle(m1, m2, true);  // Set new desired throttle for motors
}


#define motorWrite analogWrite

// Interrupt handler to send PWM signal (this triggers the actual motor control)
void pwmInterruptHandler() {
  // Apply throttle to PWM signal value
  int pwmValueMotor1 = motor1Throttle;
  int pwmValueMotor2 = motor2Throttle;

  // Write the actual PWM value to the motors
  if (lastMotor1 != pwmValueMotor1) {
    motorWrite(motorPin1, pwmValueMotor1);
    lastMotor1 = pwmValueMotor1;
  }
  if (lastMotor2 != pwmValueMotor2) {
    motorWrite(motorPin2, pwmValueMotor2);
    lastMotor2 = pwmValueMotor2;
  }
  pwmIntCalled = true;
}

// Initialize ESCs and set the throttle to 0
void initESCs() {
  // Set PWM resolution to 16 bits
  // Set the PWM frequency for each motor pin to ESC servo rate (ESC_SERVO_RATE)
  analogWriteResolution(16);
  analogWriteFrequency(motorPin1, ESC_SERVO_RATE);
  analogWriteFrequency(motorPin2, ESC_SERVO_RATE);

  pwmTimer.begin(pwmInterruptHandler, 1000000 / ESC_SERVO_RATE);
  // Set initial throttle to 0 for both motors
  setThrottle(0, 0);

  Serial.println("ESCs initialized, throttle set to 0.");
}

// for debugging/development use. add stuff you want to control via serial here.
void checkSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'p') {
     printCalibrationData();
    }
    if (cmd == 'c') {
     clearCalibrationData();
    }
    if (cmd == 'q') {
      Serial.print("Entering infinite loop so you can update or whatever. maybe enjoy the lights?\n");
      // infinite loop... might help uploading if it was in a crash loop? 
      while(1) {  delay(1); }
    }
    if(cmd == 's') // set_point accel RPS
    {
      float f1 = Serial.parseFloat();
      float f2 = Serial.parseFloat();
      Serial.print("set[f1]=f2:  accel:");
      Serial.print(f1);
      Serial.print(" RPS:");
      Serial.println(f2);
      storeCalibrationPoint(f1, f2, false);
      storeCalibrationPoint(f1, f2, true);
    }
    if(cmd == 't') // test value
    {
      float f1 = Serial.parseFloat();
      Serial.print("test[f1]: accell=");
      Serial.print(f1);
      Serial.print("  result:");
      Serial.println(calculateActualRPS(f1));
    }

  }
}

// rc_signal_is_healthy() human readable flow:
// start of by intending to say we are not healthy when we return.
// if we have seen a message in the last 500ms, 
//   then we're now going to say we're healthy when we return.
// if we haven't seen one,
//   then we'll check for one.
// if either of these ibus counters have changed since the last time we checked,
//   then we're now going to say we are healthy when we return.
//   we also update our copy of those counters.
// return our health status

bool rc_signal_is_healthy() {
  uint32_t now = millis();
  bool res = false;
  if (now - last_ibus_seen_millis < 500) {
    res = true;
  } else {
    ibus.loop();
  }
  if (last_cnt_rec != ibus.cnt_rec || last_cnt_poll != ibus.cnt_poll) {
    res = true;
    last_cnt_rec = ibus.cnt_rec;
    last_cnt_poll = ibus.cnt_poll;
    last_ibus_seen_millis = now;
  }
  return res;
}


uint32_t last_checked = (uint32_t)-1;
void wait_for_remote_signal()
{
  uint16_t pattern[] = { 
0257,0357,0457,0557,0457,0357,0257,0157,
0247,0347,0447,0547,0447,0347,0247,0147,
0237,0337,0437,0537,0437,0337,0237,0137,
0227,0327,0427,0527,0427,0327,0227,
0317,0417,0517,0417,0317,
0227,0327,0427,0527,0427,0327,0227,
0237,0337,0437,0537,0437,0337,0236,0137,
0247,0347,0447,0547,0447,0347,0247,0147,
    //0111, 0211, 0311, 0411, 0412, 0413, 0414, 0424, 0434, 0444, 0534, 0543, 0553, 0453, 0363, 0273, 0264, 0255, 0345, 0454, 0444, 0434, 0424, 0414, 0404, 0402, 0311, 0211, 
  0xffff};
  uint8_t i = 0;
  uint32_t now = millis();
  if(now - last_checked < 100)return;
  last_checked = now;
  while (!ibus.readChannel(0) || !rc_signal_is_healthy()) {
    setThrottle(0,0);
    checkSerial();
    ibus.loop();  // trying to update values
    if((i&0xf) == 0) Serial.println("Waiting for remote signal...");
    // just hopefully a pretty pattern
    setCode(pattern[i++]);
    if(pattern[i] == 0xffff)i=0;
    delay(100); 
  }
}

void initIBus() {
  // We need to use pin 20 so tell the hardward to use a different pin we don't care about to transmit.
  // if we don't, the FastLED code won't work.
  // once it works, variables again
  Serial5.setTX(39); // this is an alternate transmit pin for hardware serial port 5. 
  Serial5.setRX(21); // default  
  // Initialize the radio receiver using the IBusBM library
  ibus.begin(iBusSerialPort);
  setCode(0021); 
  Serial.println("IBus receiver initialized.");
  ibus.loop();  // get the first value?
  // Validate that commands are being received
  wait_for_remote_signal();
  Serial.println("Remote signal received.");

}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }
  checkSerial();
  
  // Initialize LED subsystem: immediate start feedback
  initLEDs();
  Serial.println("LED subsystem initialized.");

  // debugging test pattern black white cyan magenta yellow blue green red
  setRGB(32, 32, 32, 0);
  setRGB(28, 28, 32, 4);
  setRGB( 0, 28, 32, 5);
  setRGB(28,  0, 32, 6);
  setRGB(28, 28,  0, 7);
  setRGB( 0,  0, 32, 8);
  setRGB( 0, 28,  0, 9);
  setRGB(28,  0,  0, 10);

  // Initialize ESCs by sending zero throttle to both motors
  initESCs();
  Serial.println("ESCs initialized and motors stopped.");

  Serial.println("Initializing Meltybrain Robot... send 'q' now to enter infinite loop");
  delay(250);

  checkSerial();

  // Initialize IBus so we can monitor the TX without an accelerometer.
  initIBus();
  Serial.println("IBus initialized.");
  
  // Initialize accelerometer and collect calibration data
  initAccel();               // Function to initialize the accelerometer
  Serial.println("Accelerometer initialized. Maybe. See above?");

  // Additional setup steps can go here
  Serial.println("Setup complete. Ready for operation.");
}

// Function to check if throttle is nearly zero
bool isThrottleNearlyZero() {
  return powerInput < ZERO_THRESHOLD;
}

// Function to check if in tank drive mode
bool isInTankDriveMode() {
  // fixed sideways to check from left side instead of center. not sure why I missed this. ;/
  float s = (0.95 + sidewaysInput);
  return isThrottleNearlyZero() && (s < 0.25);
}

void updateInputs() {  
  // Read the vertical and horizontal direction control (channel 0 and 1)
  float vert = ibus.readChannel(0);   // Channel 0 (vertical)
  float horiz = ibus.readChannel(1);  // Channel 1 (horizontal)

  // Convert to centered values (from 1000-2000 to -1.0 to 1.0)
  stickVert  = map(vert , 1000.0, 2000.0, -1.0, 1.0); // revert to original orientation
  stickHoriz = map(horiz, 1000.0, 2000.0, -1.0, 1.0);

  // Calculate stick angle in radians as a fraction of a circle
  stickAngle = atan2(stickVert, stickHoriz) / (2 * PI); 

  // Calculate stick length (magnitude) using Pythagoras' theorem
  float sl = sqrt(sq(stickVert) + sq(stickHoriz));
  stickLength = min(sl, 1.0f);  // lets's just saturate it if it's in the corners.

  if (stickLength < 0.05) {
    stickAngle = 0;
    stickLength = 0;
  }

  // Read throttle (channel 2) and map it to 0 to 1
  float ch2 = ibus.readChannel(2);
  powerInput = map(ch2, 1000.0, 2000.0, 0.0, 1.0);

  // Read sideways input (channel 3) and map it to -1 to 1
  float ch3 = ibus.readChannel(3);
  sidewaysInput = map(ch3, 1000, 2000, -1.0, 1.0);

  // Read radius input (channel 4) and map it to 5mm to 30mm
  radiusInput = ibus.readChannel(4);
  radiusSize = map(radiusInput, 1000, 2000, 0.010, 0.025);  // Result in meters (range 5 mm to 30 mm) 

  // Read LED offset input (channel 5).
  float ch5 = ibus.readChannel(5);
  headingOffset = map(ch5, 1000.0, 2000.0, -0.5, 0.5);

  // left and right switches control if we should reverse M1 and M2.
  // DO NOT SWITCH THESE WHILE MOVING. Probably. It might be cool, but it might blow up your robot.
  // This is to try mitigate "plugged in a motor the wrong way round" issues.
  reverseM1 = (ibus.readChannel(6) > 1250);
  reverseM2 = (ibus.readChannel(9) > 1250);

  int ch8 = ibus.readChannel(8);
  sw3pos = abs(ch8 - 1000) / 500;  // assuming it doesn't go too far below 1000, this should work
  int ch7 = ibus.readChannel(7);
  swReverseText = ch7 > 1500 ? 1 : 0;

  if(!rc_signal_is_healthy())
  {
    stickAngle = 0;
    stickLength = 0;
    powerInput = 0;
    pidStateM1.targetRPM = 0.0f;
    pidStateM2.targetRPM = 0.0f;
    
  }

}

extern volatile float accelMag;
extern sensors_event_t sensor;

void displayState() {
  static uint32_t lastIdleMessage = 0;
  uint32_t now = millis();
  if (now - lastIdleMessage > 12) {
    lastIdleMessage = now;
//Serial.print("\t");
    Serial.print("accelMag: ");
    Serial.print(accelMag / SENSORS_GRAVITY_STANDARD);
//Serial.print("\t");
    Serial.print("  accelAngle: ");
    Serial.print(accelAngle);
//Serial.print("\t");
    Serial.print("  accel.x: ");
    Serial.print(sensor.acceleration.x);
//Serial.print("\t");
    Serial.print("  accel.y: ");
    Serial.print(sensor.acceleration.y);
//Serial.print("\t");
    Serial.print("  accel.z: ");
    Serial.println(sensor.acceleration.z);
    Serial.print("  m1Throttle: ");
    Serial.print(motor1Throttle);
    Serial.print("  m2Throttle: ");
    Serial.print(motor2Throttle);
    Serial.print("  pwmIntCalled: ");
    Serial.print(pwmIntCalled);
    Serial.print("  stickVert: ");
    Serial.print(stickVert);
    Serial.print("  stickHoriz: ");
    Serial.print(stickHoriz);
    Serial.print("  stickAngle: ");
    Serial.print(stickAngle);
    Serial.print("  stickLength: ");
    Serial.print(stickLength);
    Serial.print("  power: ");
    Serial.print(powerInput);
    Serial.print("  sidewaysInput: ");
    Serial.print(sidewaysInput);
    Serial.print("  radiusInput: ");
    Serial.print(radiusInput);
    Serial.print("  radiusSize: ");
    Serial.print(radiusSize * 1000.0f);
    Serial.print("mm  headingOffset: ");
    Serial.print(headingOffset);
    Serial.print("  reverseM1: ");
    Serial.print(reverseM1);
    Serial.print("  reverseM2: ");
    Serial.print(reverseM2);
    Serial.print("  sw3pos: ");
    Serial.print(sw3pos);
    Serial.println();
  }
}

// Function to calculate revolutions per second based on spin acceleration
float calculateRPS() {
  // Get the spin acceleration (centripetal force)
  float spinAccel = accelMag;
  if(spinAccel < 1) spinAccel  = 1;
  // Use the dynamically updated radiusSize as the spin radius (in meters)
  float spinRadius = radiusSize;  // radiusSize is in meters. expected range is from at least 1mm to about 10cm.

  if (spinRadius < 0.001) spinRadius = 0.001; // this should only happen if it's starting up, I think. this will stop divide by zero.
  // Calculate angular velocity (omega) in radians per second
  float omega = sqrt(spinAccel / spinRadius);

  // Convert angular velocity to RPS
  float rps = (omega) / (2 * PI);

  return rps;  // Return the calculated RPS
}

unsigned long usRevStartTime = 0;  // Time in microseconds when the revolution started
unsigned long usPrevRevStartTime = 0;               // Store the last timestamp

// draws RPM on the bot. hopefully. it's only 3 digits now so it might fit.
void updateLEDDisplay(float phase, float rps)
{
    uint8_t columnData[8];  // Assuming 8 LEDs per column
    
  // Display RPM in first half of rotation in blue
    uint16_t rpm = (uint16_t)(rps * 60.0f);  // Convert RPS to RPM
    getColumnData(rpm, phase * 1.5f, swReverseText, columnData);
    for (int i = 0; i < 5; i++) { // forcing 5 coz that's the actual number to write.
        if (columnData[i]) {
            setRGB(128, 0, 255, i + 3);  // Blue for RPM
        } else {
            setRGB(0, 0, 0, i + 3);    // Off if no pixel
        }
    }
}

// DONE: refactored handleMeltybraindrive()
// I asked claude.ai to refactor it for me, specifically telling it to make sensible sub-functions for each of the following units:
//  the parts related to getting the phase, the parts related to the throttle control, and the parts related to the LEDs
// This is the result. I like it, at least at first glance.

/**
 * Updates the robot's phase based on elapsed time and rotation speed
 * 
 * @param rps Current revolutions per second
 * @param now Current time in microseconds
 * @return Updated revolution duration in microseconds
 */
unsigned long updatePhase(float rps, unsigned long now) {
  // Calculate time since last update
  unsigned long deltaTime = now - phaseState.lastPhaseUpdate;
  phaseState.lastPhaseUpdate = now;
  
  // Calculate the current revolution period based on rps
  unsigned long updatedRevDuration = (unsigned long)(1000000.0f / rps);
  
  // Increment continuous phase based on elapsed time, scaled by the current revolution period
  phaseState.continuousPhase += (float)deltaTime / updatedRevDuration;
  
  // Handle phase wrap-around
  if(phaseState.continuousPhase > 1.0f) {
    phaseState.usRevStartTime = now; 
    phaseState.continuousPhase = fmod(phaseState.continuousPhase, 1.0f);
  // should this use the fractional part (timeSinceRevStart/revDuration) to project the actual start time more accurately? assumption is that will be called regularly enough to make this close enough.
  }
  
  // Update global angular position
  angularPosition = phaseState.continuousPhase;
  
  return updatedRevDuration;
}

// Modify calculatePIDOutput to handle bidirectional control
float calculatePIDOutput(PIDState &pidState, float currentRPM, unsigned long now) {
  // Time since last PID update in seconds
  float dt = (now - pidState.lastPIDUpdate) / 1000000.0f;
  pidState.lastPIDUpdate = now;
  
  // Prevent division by zero and unreasonable dt values
  if (dt < 0.001f || dt > 0.1f) dt = 0.001f;

  if (pidState.targetRPM == 0.0f) return 0; // do not optimize for being stopped yet.

  // Calculate error between target and current RPM
  float error = pidState.targetRPM - currentRPM;

  // Calculate integral term with anti-windup
  pidState.errorIntegral += error * dt;
  if (pidState.errorIntegral > pidConstants.MAX_INTEGRAL) 
    pidState.errorIntegral = pidConstants.MAX_INTEGRAL;
  if (pidState.errorIntegral < -pidConstants.MAX_INTEGRAL) 
    pidState.errorIntegral = -pidConstants.MAX_INTEGRAL;
  
  // Calculate derivative term
  float errorDerivative = 0.0f;
  if (pidState.lastError != 0.0f || error != 0.0f) {
    errorDerivative = (error - pidState.lastError) / dt;
  }
  pidState.lastError = error;
  
  // Calculate PID output
  float pidOutput = (pidConstants.kP * error) + 
                    (pidConstants.kI * pidState.errorIntegral) + 
                    (pidConstants.kD * errorDerivative);
  
  // Apply minimum throttle to overcome friction
  float baseLevel = pidOutput;
  
  // if we asked for less than 100 RPM, do nothing.
  if (pidState.targetRPM <= (ZERO_THRESHOLD*pidConstants.MAX_RPM) ) {
    baseLevel = 0.0f;
  } else {
    // Apply minimum throttle magnitude
    float minThrottleSign = (pidState.targetRPM > 0) ? 1.0f : -1.0f;
    if (fabs(baseLevel) < pidConstants.MIN_THROTTLE) {
      baseLevel = minThrottleSign * pidConstants.MIN_THROTTLE;
    }
  }
  
  // Clamp the base throttle level
  if (baseLevel > pidConstants.OUTPUT_CAP) baseLevel = pidConstants.OUTPUT_CAP;
  if (baseLevel < -pidConstants.OUTPUT_CAP) baseLevel = -pidConstants.OUTPUT_CAP;
  
  return baseLevel;
}

// Update calculateMotorThrottles to handle negative base levels
struct ThrottleValues calculateMotorThrottles(float baseLevel, unsigned long updatedRevDuration, unsigned long usTimeSinceZero) {
  // Calculate the adjusted heading
  float adjustedHeading = headingOffset + stickAngle;
  adjustedHeading = fmod(adjustedHeading, 1.0);
  if (adjustedHeading < 0.0) {
    adjustedHeading += 1.0;
  }
  
  // Calculate delay durations for phase-based control
  float usM1DelayDuration = adjustedHeading * updatedRevDuration;
  float m2PhaseOffset = 0.5; // TODO offset by fraction of stickLength so 1 = 0.5
  float usM2DelayDuration = usM1DelayDuration + m2PhaseOffset * updatedRevDuration;
  if (usM2DelayDuration > updatedRevDuration) {
    usM2DelayDuration -= updatedRevDuration;
  }
  
  // Calculate phases for cosine-based throttle modulation
  float ph1 = ((usTimeSinceZero - usM1DelayDuration) / (float)updatedRevDuration) * M_PI * 2.0f;
  float ph2 = ((usTimeSinceZero - usM2DelayDuration) / (float)updatedRevDuration) * M_PI * 2.0f;
  
  
  float cos_ph1 = cos(ph1);
  g_cos_ph1 = cos_ph1;
  float cos_ph2 = cos(ph2);
  
  // Calculate throttle modulation parameters
  float speedLen = stickLength;
  float mixFrac = 0.75f; // changed from 0.25f in attempt to get some useful translation...

  cos_ph1 = signbit(cos_ph1) ? -1 : 1;
  cos_ph2 = signbit(cos_ph2) ? -1 : 1;

  ThrottleValues throttles;
  throttles.th1 = (cos_ph1 * mixFrac * speedLen + 1.0f) * baseLevel; // changed to be a proportion up/down - TODO: remove speedLen when using phase based speed control
  throttles.th2 = (cos_ph2 * mixFrac * speedLen + 1.0f) * baseLevel;
  
  throttles.th1 = fmax(-1.0f, fmin(1.0f, throttles.th1));
  throttles.th2 = fmax(-1.0f, fmin(1.0f, throttles.th2));
  
  return throttles;
}


extern volatile float accelMag;
float maxAccelMag = 0;
float minAccelMag = 1e7;

/**
 * Updates LED display and indicators based on current state
 * 
 * @param cos_ph1 Cosine of first motor phase
 * @param currentRPM Current RPM of the robot
 */
void updateLEDIndicators(float cos_ph1, float currentRPM) {
  // Set LED color based on motor 1 phase
  uint8_t r = cos_ph1 < 0.5 && cos_ph1 > -0.5 ? 255 : 0;
  uint8_t g = cos_ph1 < -0.7071 ? 255 : 0;
  uint8_t b = cos_ph1 >  0.7071 ? 255 : 0; // > 0.7071 ? 255 : 0;
  setRGB(r, g, b, 0);
  
  // Draw 8 colors in a circle based on angular position
  int pp = (int)fmod(angularPosition * 8.0, 8.0);
  r = pp & 4 ? 128 : 0;
  g = pp & 2 ? 128 : 0;
  b = pp & 1 ? 128 : 0;
  setRGB(r, g, b,  8);
  setRGB(r, g, b,  9);
  setRGB(r, g, b, 10);
  
  // Update LED display with current position, RPM and target percentage
//  updateLEDDisplay(angularPosition, currentRPM); // let's just get RPM wrapped nicely?  // let's not do this right now... 

// display trans pride flag for Neether
// you may wish to update or change the colors.
  setRGB( 8, 8,48, 3); // light blue
  setRGB(48, 8, 8, 4); // pink
  setRGB(32,32,32, 5); // white
  setRGB(48, 8, 8, 6); // pink
  setRGB( 8, 8,48, 7); // light blue

  // display live scaled accelerometer data
  // these dots are drawn last.
  // the dots distance from the center coresponds to the accelerometer force. high forces, relative to the highest levels, are further from the center.
  // there are 3 trails: the brightest, the dimmer, and the dimmest. These are the current value, the previous value, and the previous to previous value. they are drawn Bright, sorta bright, and dim.
  if(maxAccelMag<accelMag)maxAccelMag=accelMag;
  if(minAccelMag>accelMag)minAccelMag=accelMag;
  float accelRange = maxAccelMag - minAccelMag;
  if(accelRange == 0)accelRange = 1;
  float scaledAccelMag = (accelMag - minAccelMag) / accelRange;
  static int prevAccelLED = 0;
  static int prevPrevAccelLED = 0;
  int accelLED = numled * scaledAccelMag;
  if(prevPrevAccelLED)
  {
   setRGB(64,64,64, prevAccelLED);
  }
  if(prevAccelLED)
  {
   setRGB(128,128,128, prevAccelLED);
  }
  if(accelLED) // don't write if acceleration is really low.
  {
   setRGB(255,255,255, accelLED);
  }
  prevPrevAccelLED = prevAccelLED;
  prevAccelLED = accelLED;

//  updateLEDs();
}

/**
 * Main function to handle melty brain driving mode
 */
void handleMeltybrainDrive() {
  static uint32_t numCalls = 0;
  
  // Initialize timing variables if this is the first call
  if (numCalls == 0) {
    phaseState.lastPhaseUpdate = micros();
    pidStateM1.lastPIDUpdate = micros();
    pidStateM2.lastPIDUpdate = micros();
  }
  
  while (true) {
    numCalls++;
    
    // Check for signal health and update inputs
    if (!rc_signal_is_healthy()) {
      stickLength = 0.0;
      powerInput = 0.0;
      pidStateM1.targetRPM = 0.0f;
      pidStateM2.targetRPM = 0.0f;
      setThrottle(0,0); 
      return;
    } else {
      updateInputs();
    }
    
    // Get current time
    unsigned long now = micros();
    
    // sw3Pos is one of { 0, 1, 2 }; 
    // 0: use LUT with accelMag for rps
    // 1: use radius to calculate rps
    // 2: use current accel and radius to generate RPS value and save it indexed by current accel for use in interpolation.
    //    if you stay in this position while throttle is zero for longer than 5 seconds, it will erase all current data.
    //    it will blink wildly to warn you, giving you plenty of time to change state. see loop()

    float rps = calculateActualRPS(accelMag);
    if(sw3pos != 0) rps = calculateRPS();
    storeCalibrationPoint(accelMag, rps, sw3pos == 2); 
   
    // Calculate current rotational speed
    float currentRPM = rps * 60.0f; // Convert RPS to RPM
    
    // Update phase tracking
    unsigned long updatedRevDuration = updatePhase(rps, now);
    
    // Calculate time since last revolution start
    unsigned long usTimeSinceZero = now - phaseState.usRevStartTime;

    ThrottleValues modulatedThrottle = calculateMotorThrottles(powerInput, updatedRevDuration, usTimeSinceZero);

    pidStateM1.targetRPM = modulatedThrottle.th1 * pidConstants.MAX_RPM;
    pidStateM2.targetRPM = modulatedThrottle.th2 * pidConstants.MAX_RPM;

    // TODO: setThrottle(modulatedThrottle.th1, modulatedThrottle.th2);
    // TODO: comment out calculatePIDOutput() calls
    // TODO: IS THIS A GOOD IDEA? No.

    float baseLevelM1, baseLevelM2;
    // baseLevelM1 = calculatePIDOutput(pidStateM1, currentRPM, now);
    // baseLevelM2 = calculatePIDOutput(pidStateM2, currentRPM, now);
    baseLevelM1 = modulatedThrottle.th1 * 0.5;
    baseLevelM2 = modulatedThrottle.th2 * 0.5;

    setThrottle(baseLevelM1, -baseLevelM2);
    
    // Update LED display
    updateLEDIndicators(g_cos_ph1, currentRPM);
    
    // Check for exit conditions
    static unsigned long lastBreakCheck = 0;
    if (now - lastBreakCheck > 1000000) { // 1 second
        lastBreakCheck = now;
        break;
    }
    if(isInTankDriveMode()) break;
    if (!rc_signal_is_healthy()) break;
  }  
}

void loop() {
  // Update the IBus object for Teensy
  checkSerial();
  ibus.loop();
  wait_for_remote_signal();
  updateInputs();
  if (isInTankDriveMode()) {
    setCode(0202);
    
    handleTankDrive();
    delay(5);  // give us time to do _something_. Driving around doesn't
    displayState();
    return;
  }

  static uint32_t configResetTriggered;
  if (isThrottleNearlyZero()) {
    setRGB(0x70, 0x70, 0x00, 5); // "zero throttle" detected so LED[9] = yellow
    setThrottle(0, 0);
    if (sw3pos != 2)
    {
      configResetTriggered = 0;
    }
    else
    {
      if(configResetTriggered == 0)
      {
        configResetTriggered = millis();
      }
      setCode(0700 & millis());
      uint32_t elapsed = millis() - configResetTriggered;
      if(elapsed > 1000)
      {
        setCode(0720);
      }
      if(elapsed > 2000)
      {
        setCode(0730);
      }
      if(elapsed > 3000)
      {
        setCode(0740);
      }
      if(elapsed > 4000)
      {
        setCode(0640);
      }
      if(elapsed > 5000)
      {
        setCode(0550);
      }
      if(elapsed > 6000)
      {
        setCode(0711);
      }
      if(elapsed > 7000)
      {
        setCode(0007 & millis());
        delay(100);
      }
      if(elapsed > 8000)
      {
        clearCalibrationData();
        elapsed = 0;
      }
      delayMicroseconds(2137);
      return;
    }
  } else {
    setCode(0);  // all off? this may need to change
  }
  handleMeltybrainDrive();
  
  displayState();
}
