#include "Config.h"          // must be first — defines IR_MODE, arena constants, etc.
#include "Autonomy.h"
#include "Accelerometer.h"
#include "AccelCalibration.h"
#include "Magnetometer.h"
#include "AngleEstimator.h"
#include "Motor.h"
#include "../lib/teensyshot/ESCCMD.h"
#include "../lib/teensyshot/DSHOT.h"
#include "Receiver.h"
#include "Robot.h"
#include "Telemetry.h"
#include "IR.h"
#if IR_MODE == 1
#include "IRArena.h"
#endif
#include <Wire.h>


AccelerometerManager     accelerometers;
AccelCalibrationManager  accelCal;
MagnetometerTracker      mag;
AngleEstimator           estimator(accelerometers, mag);
MotorManager             motors;
CrsfReceiver             rc;
Robot                    robot(estimator, motors);
Telemetry                telemetry(estimator, mag, motors);
Autonomy                 autonomy;
#if IR_MODE == 0
IRBeaconTracker          beacons;
IRSweep                  sweep;
#else
IRArenaTracker           arena;
#endif

unsigned previousTime = 0;
unsigned currentTime = 0;

// ─── Accelerometer calibration mode ──────────────────────────────────────────
// How to enter:  hold CH6 (channels[5]) above 1700 µs for 2 seconds while
//                throttle (CH3, channels[2]) is at minimum (< 1100 µs).
// While in cal mode the main drive loop does NOT run (robot stays still).
// RC channel assignments in this block — adjust to match your transmitter:
static constexpr uint8_t  CAL_ENTRY_CH   = 5;   // 0-based channel index (CH6)
static constexpr uint8_t  CAL_SAVE_CH    = 5;   // same channel — held=commit, tap=factor change
static constexpr uint8_t  CAL_FACTOR_CH  = 0;   // CH1: left/right stick adjusts working factor
static constexpr uint8_t  CAL_ZERO_CH    = 1;   // CH2: pull low + hold 2 s → zero-G capture

static bool     inCalMode          = false;
static uint32_t calEntryStartMs    = 0;  // when CAL_ENTRY_CH first went high (entry path)
static uint32_t calSaveStartMs     = 0;  // when the commit/clear gesture started
static uint32_t calExitStartMs     = 0;  // when the exit gesture (ch low) started
static uint32_t calZeroStartMs     = 0;  // when the zero-G hold gesture started
static uint32_t lastCalPrintMs     = 0;

/**
 * Handle one tick of calibration mode.
 * Called from loop() whenever inCalMode == true.
 * Uses proper start-timestamp tracking so hold durations are accurate regardless
 * of loop speed.
 *
 * Stick layout (all thresholds in µs, RC_MID = 1500):
 *   CAL_FACTOR_CH > 1700  → increment working factor by ACCEL_CAL_FACTOR_STEP
 *   CAL_FACTOR_CH < 1300  → decrement working factor
 *   CAL_ZERO_CH   < 1200, held 2 s → captureZeroG() (robot must be still)
 *   CAL_ENTRY_CH  > 1700, held 2 s → commit current point to both tables
 *   CAL_ENTRY_CH  > 1700, held 4 s → clear all tables (zero offsets untouched)
 *   CAL_ENTRY_CH  < 1300, held 2 s → EXIT calibration mode and save to EEPROM
 */
void handleCalibration(const uint16_t* ch) {
    uint32_t now = millis();

    // ── Working-factor adjustment ─────────────────────────────────────────────
    // Rate-limited to ~12 taps/s so one full deflection changes the factor by
    // ~0.06 per second — fast enough to tune but not so fast it overshoots.
    static uint32_t lastFactorTickMs = 0;
    if (now - lastFactorTickMs >= 80) {
        if      (ch[CAL_FACTOR_CH] > 1700) accelCal.adjustFactor(+ACCEL_CAL_FACTOR_STEP);
        else if (ch[CAL_FACTOR_CH] < 1300) accelCal.adjustFactor(-ACCEL_CAL_FACTOR_STEP);
        lastFactorTickMs = now;
    }

    // ── Zero-G capture: CH2 back, held 2 s ───────────────────────────────────
    if (ch[CAL_ZERO_CH] < 1200) {
        if (calZeroStartMs == 0)          calZeroStartMs = now;
        if (now - calZeroStartMs >= 2000) {
            accelerometers.captureZeroG();  // blocking ~400 ms, robot must be stationary
            accelCal.resetWorkingFactor();
            calZeroStartMs = UINT32_MAX;    // prevent re-trigger until stick released
        }
    } else {
        calZeroStartMs = 0;
    }

    // ── Commit / clear: CAL_ENTRY_CH held high ───────────────────────────────
    if (ch[CAL_ENTRY_CH] > 1700) {
        calExitStartMs = 0;  // can't exit while holding high
        if (calSaveStartMs == 0) calSaveStartMs = now;
        uint32_t held = now - calSaveStartMs;

        if (held >= 4000 && calSaveStartMs != UINT32_MAX) {
            // 4-second hold → clear all correction tables.
            accelCal.clearAllTables();
            accelCal.resetWorkingFactor();
            calSaveStartMs = UINT32_MAX;    // prevent re-trigger until released
            Serial.println("[CalMode] Tables cleared — zero offsets preserved.");
        } else if (held >= 2000 && calSaveStartMs != UINT32_MAX) {
            // 2-second hold → commit current G readings + workingFactor.
            Vec3d raw1 = accelerometers.fetchRawXYZ1();
            Vec3d raw2 = accelerometers.fetchRawXYZ2();
            accelCal.commitPoint(
                (float)raw1.x(), (float)raw2.x(),
                (float)raw1.y(), (float)raw2.y(),
                (float)raw1.z(), (float)raw2.z()
            );
            accelCal.resetWorkingFactor();
            calSaveStartMs = UINT32_MAX;    // prevent re-trigger at 4 s
        }
    } else {
        calSaveStartMs = 0;
    }

    // ── Exit: CAL_ENTRY_CH held low 2 s ──────────────────────────────────────
    if (ch[CAL_ENTRY_CH] < 1300) {
        if (calExitStartMs == 0) calExitStartMs = now;
        if (now - calExitStartMs >= 2000) {
            inCalMode = false;
            accelCal.save();
            Serial.println("[AccelCal] *** Calibration mode exited — saved to EEPROM. ***");
            calExitStartMs = 0;
            return;
        }
    } else if (ch[CAL_ENTRY_CH] <= 1700) {
        calExitStartMs = 0;
    }

    // ── Periodic status print (every 500 ms) ─────────────────────────────────
    if (now - lastCalPrintMs >= 500) {
        lastCalPrintMs = now;

        // Drift rate: measures how fast the integrated angle diverges from the
        // magnetometer heading.  This is the calibration signal — drive it to ≈0.
        //   drift > 0 → ω is over-estimated → reduce workingFactor (← stick left)
        //   drift < 0 → ω is under-estimated → increase workingFactor (→ stick right)
        float driftDegPerSec = 0.0f;
        if (mag.isAngleValid()) {
            static float    prevMagAngle = 0.0f;
            static float    prevEstAngle = 0.0f;
            static uint32_t prevPrintMs  = 0;
            float dt = (now - prevPrintMs) * 0.001f;
            if (dt > 0.1f && prevPrintMs != 0) {
                float magDelta = mag.getAngle(micros()) - prevMagAngle;
                float estDelta = estimator.getAngle()   - prevEstAngle;
                // Wrap both deltas to [-π, π] to handle wrap-around at 0/2π.
                auto wrap = [](float a) {
                    while (a >  PI) a -= 2*PI;
                    while (a < -PI) a += 2*PI;
                    return a;
                };
                driftDegPerSec = wrap(estDelta - magDelta) * (180.0f / PI) / dt;
            }
            prevMagAngle = mag.getAngle(micros());
            prevEstAngle = estimator.getAngle();
            prevPrintMs  = now;
        }

        Serial.printf("[CalMode] factor=%+.4f  ω=%.1frad/s  G0=%+.2fg  G1=%+.2fg  drift=%+.1f°/s\n",
                      accelCal.workingFactor(),
                      estimator.getOmega(),
                      (float)accelerometers.fetchXYZ1().x(),
                      (float)accelerometers.fetchXYZ2().x(),
                      driftDegPerSec);
        Serial.println("  [←→]=factor  [CH2 back 2s]=zero-G  [CH6 hold 2s]=commit  [CH6 hold 4s]=clear  [CH6 fwd 2s]=exit+save");
    }
}

uint16_t channels[CRSF_NUM_CHANNELS];
CrsfStatus status;

const int RED_LED_PIN = 5;   // was 8 — pin 8 is DShot ch2 output, moved to free pin 5
const int GREEN_LED_PIN = 6;

void setup() {
    Serial.begin(9600);
    delay(1000);
    if (CrashReport) {
        Serial.print(CrashReport); // Print any previous crash info
    }
    Serial.println("--- SETUP START ---");
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);

    Serial.println("Loading accelerometer calibration from EEPROM...");
    accelCal.load();
    accelerometers.attachCalibration(&accelCal);
    Serial.println("...Calibration loaded.");

    Serial.println("Initializing Accelerometers...");
    Wire.begin();
    Wire.setClock(400000);   // 400 kHz fast mode
    accelerometers.init(0x18, 0x19);
    Serial.println("...Accelerometers Initialized.");

    Serial.println("Initializing Receiver...");
    rc.init();
    Serial.println("...Receiver Initialized.");

    Serial.println("Initializing Magnetometer...");
    if (!mag.init()) {
        Serial.println("WARNING: Magnetometer not found — running on accelerometer only.");
    } else {
        Serial.println("...Magnetometer Initialized.");
    }

#if IR_MODE == 0
    Serial.println("Initializing IR beacons...");
    beacons.init();       // attaches interrupts on IR_BEACON_A_PIN and IR_BEACON_B_PIN
    beacons.calibrate();  // first spin sets each beacon's phase automatically
    sweep.init();         // sets up 36 kHz PWM emitter and sweep receiver interrupt
    sweep.enable();       // start emitting — comment out if not using active sweep
    Serial.println("...IR beacons Initialized.");
#else
    Serial.println("Initializing IR arena tracker...");
    arena.init();
    arena.setSquareArena(ARENA_SIZE_M);
    arena.enable();
    Serial.println("...IR arena tracker Initialized.");
#endif

    Serial.println("Initializing Telemetry link...");
    telemetry.init();
    Serial.println("...Telemetry link Initialized.");

    Serial.println("Capturing zero-G baseline — keep robot still...");
    accelerometers.captureZeroG();
    Serial.println("...Zero-G capture complete.");

    // Arm motors last — everything above takes >1 s, giving the ESC plenty of
    // boot time. The main loop starts immediately after so ESCCMD_tic() is
    // called every iteration and the 250 ms watchdog never fires.
    Serial.println("Initializing Motors & Arming ESCs...");
    motors.init();
    Serial.println("...Motors Armed.");

    Serial.println("--- SETUP COMPLETE, entering main loop ---");
    previousTime = micros();
}


void loop() {
    ESCCMD_tic();   // must be called every loop — sends queued DShot frames and services the ESC watchdog
    rc.fetch(channels, &status);
    if (rc.isLost()) {
        Serial.println("CRSF signal lost! Halting.");
        while (1) {
            robot.move(0, 0, 0);
        }
    }

    // ── Calibration mode entry ────────────────────────────────────────────────
    // Hold CH6 (channels[5]) above 1700 µs for 2 s while throttle is low (< 1100).
    if (!inCalMode) {
        if (channels[CAL_ENTRY_CH] > 1700 && channels[2] < 1100) {
            if (calEntryStartMs == 0) calEntryStartMs = millis();
            if (millis() - calEntryStartMs >= 2000) {
                inCalMode = true;
                calEntryStartMs = 0;
                calSaveStartMs  = 0;
                calExitStartMs  = 0;
                calZeroStartMs  = 0;
                lastCalPrintMs  = 0;
                accelCal.resetWorkingFactor();
                Serial.println("[AccelCal] *** Calibration mode entered. ***");
                accelCal.printTable();
            }
        } else {
            calEntryStartMs = 0;
        }
    }

    // ── Calibration mode active — skip drive logic ────────────────────────────
    if (inCalMode) {
        accelerometers.refresh();
        mag.update(micros());
        estimator.update(micros());
        handleCalibration(channels);
        return;
    }
     Serial.print("Receiver: ");
     Serial.print("Ch 1: ");
     Serial.print(channels[0]);
     Serial.print(" Ch 2: ");
     Serial.print(channels[1]);
     Serial.print(" Ch 3: ");
     Serial.println(channels[2]);
    currentTime = micros();
    accelerometers.refresh();       // 1. read accelerometer
    mag.update(currentTime);        // 2. read magnetometer
    estimator.update(currentTime);  // 3. fuse — must come after both sensors
    Serial.printf(">omega:%.2f\n", estimator.getOmega());
    static uint32_t lastDiagMs = 0;
    if (millis() - lastDiagMs >= 1000) {
        lastDiagMs = millis();
        Serial.printf("[DSHOT] ISR0=%lu  ISR1=%lu\n",
                      (unsigned long)DSHOT_isr_count[0],
                      (unsigned long)DSHOT_isr_count[1]);
    }

#if IR_MODE == 0
    // 4. IR heading correction — runs after the estimator so it has fresh
    //    angle/omega, and before robot.move() so the robot acts on the
    //    corrected angle.
    beacons.update(currentTime, estimator.getAngle(), estimator.getOmega());
    if (beacons.hasHeadingFix()) {
        estimator.correctAngle(IR_SNAP_GAIN * beacons.getHeadingError());
        beacons.clearHeadingFix();
    }
    // Keep the sweep beacon filter in sync with the latest measured angles so
    // confirmed beacon directions are excluded from unknown-target hits.
    if (beacons.seenA()) sweep.setBeaconAPhase(beacons.getLastAngleA());
    if (beacons.seenB()) sweep.setBeaconBPhase(beacons.getLastAngleB());
    sweep.update(currentTime, estimator.getAngle(), estimator.getOmega());
#else
    // 4. IR arena heading + position — wall reflections replace external beacons.
    arena.update(currentTime, estimator.getAngle(), estimator.getOmega());
    if (arena.hasHeadingFix()) {
        estimator.correctAngle(ARENA_SNAP_GAIN * arena.getHeadingError());
        arena.clearHeadingFix();
    }
#endif

    // ── Autonomy dispatch ─────────────────────────────────────────────────────
    // Gather position + opponent + wall data then hand off to the autonomy
    // system, which picks the right mode (TANK/MELTY/ASSISTED/AUTO) based on
    // the RC switch and returns a DriveCommand.
#if IR_MODE == 1
    float auto_wall_dist    = arena.getNearestWallDist();
    float auto_wall_bearing = arena.getNearestWallBearing();
    bool  auto_opp_valid    = arena.getOpponentHitCount() > 0;
    float auto_opp_bearing  = auto_opp_valid ? arena.getOpponentHitAngle(0) : 0.0f;
    float auto_pos_x        = arena.getX();
    float auto_pos_y        = arena.getY();
    bool  auto_pos_valid    = arena.hasPosition();
#else
    float auto_wall_dist    = 999.0f;
    float auto_wall_bearing = 0.0f;
    bool  auto_opp_valid    = false;
    float auto_opp_bearing  = 0.0f;
    float auto_pos_x        = 0.0f;
    float auto_pos_y        = 0.0f;
    bool  auto_pos_valid    = false;
#endif

    DriveCommand cmd = autonomy.update(
        channels,
        estimator.getOmega(),
        auto_pos_x, auto_pos_y, auto_pos_valid,
        auto_opp_bearing, auto_opp_valid,
        auto_wall_dist, auto_wall_bearing
    );

    if (cmd.isTank) {
        motors.on(cmd.left, cmd.right);
    } else {
        robot.move(cmd.ch1_us, cmd.ch2_us, cmd.ch3_us);
    }

#if IR_MODE == 1
    // Feed position into telemetry every loop so the packet is always current.
    if (arena.hasPosition()) {
        telemetry.setPosition(arena.getX(), arena.getY());
    } else {
        telemetry.clearPosition();
    }
    // Feed nearest opponent bearing (first hit from the sweep ring buffer).
    if (arena.getOpponentHitCount() > 0) {
        telemetry.setOpponent(arena.getOpponentHitAngle(0));
    } else {
        telemetry.clearOpponent();
    }
#endif

    telemetry.update(currentTime, channels, rc.isLost());  // stream to ESP32 (rate-limited)

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
    Serial.println(currentTime / 1000000.0, 6);
    Serial.print("Time since previous: ");
    Serial.println((currentTime - previousTime) / 1000000.0, 6);

    previousTime = currentTime;
}